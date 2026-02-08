import sys
import sdl2
from PIL import Image


class UiBoardSim:
    OLED_W, OLED_H = 128, 64

    def __init__(self, scale=4) -> None:
        """Same interface as UiBoard, but use SDL2 backend"""

        self.is_inverted = False
        self.brightness = 8
        self.scale = scale

        # Init SDL2 window and texture
        if sdl2.SDL_Init(sdl2.SDL_INIT_VIDEO) != 0:
            sys.stderr.write(f"SDL_Init error: {sdl2.SDL_GetError().decode()}\n")
            sys.exit(1)

        window = sdl2.SDL_CreateWindow(
            b"ui_board simulation",
            sdl2.SDL_WINDOWPOS_CENTERED,
            sdl2.SDL_WINDOWPOS_CENTERED,
            UiBoardSim.OLED_W * scale,
            UiBoardSim.OLED_H * scale,
            sdl2.SDL_WINDOW_SHOWN,
        )
        if not window:
            sys.stderr.write(f"SDL_CreateWindow error: {sdl2.SDL_GetError().decode()}\n")
            sys.exit(1)

        self.renderer = sdl2.SDL_CreateRenderer(
            window,
            -1,
            sdl2.SDL_RENDERER_ACCELERATED | sdl2.SDL_RENDERER_PRESENTVSYNC,
        )
        if not self.renderer:
            sys.stderr.write(f"SDL_CreateRenderer error: {sdl2.SDL_GetError().decode()}\n")
            sys.exit(1)

        # Create a *streaming* texture that we will update each frame.
        # Pixel format: SDL_PIXELFORMAT_RGB24 matches Pillow's "RGB" raw bytes.
        self.texture = sdl2.SDL_CreateTexture(
            self.renderer,
            sdl2.SDL_PIXELFORMAT_RGB24,
            sdl2.SDL_TEXTUREACCESS_STREAMING,
            UiBoardSim.OLED_W,
            UiBoardSim.OLED_H,
        )
        if not self.texture:
            sys.stderr.write(f"SDL_CreateTexture error: {sdl2.SDL_GetError().decode()}\n")
            sys.exit(1)

    def set_inverted(self, v=False):
        self.is_inverted = v

    def set_brightness(self, val=16):
        if val < 0:
            val = 0
        if val > 16:
            val = 16
        self.brightness = val

    def set_gamma(self, max_value=180, vals=None):
        pass

    def send_img(self, img: Image.Image):
        # arr = np.array(img, dtype=np.uint8)
        # arr4 = arr >> 4  # 8-bit → 4-bit
        rgb_bytes = img.convert("RGB").tobytes()

        # Lock texture, copy new pixel data, then unlock
        # (SDL_UpdateTexture does the lock/unlock internally)
        sdl2.SDL_UpdateTexture(
            self.texture,
            None,  # update whole texture
            rgb_bytes,
            UiBoardSim.OLED_W * 3,  # pitch = bytes per row (RGB24 → 3*width)
        )

        # --- clear, copy texture (with scaling), present --------------------
        sdl2.SDL_SetRenderDrawColor(self.renderer, 0, 0, 0, 255)  # black background
        sdl2.SDL_RenderClear(self.renderer)

        # Destination rectangle – scaled up for visibility
        dst_rect = sdl2.SDL_Rect(0, 0, UiBoardSim.OLED_W * self.scale, UiBoardSim.OLED_H * self.scale)
        sdl2.SDL_RenderCopy(self.renderer, self.texture, None, dst_rect)
        sdl2.SDL_RenderPresent(self.renderer)
