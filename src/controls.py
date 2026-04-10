# ─────────────────────────────────────────────────────────────────────────────
# controls.py — Xbox controller (XInput) and pygame joystick input handling
# ─────────────────────────────────────────────────────────────────────────────

import sys

try:
    import ctypes
    XINPUT_AVAILABLE = True
except ImportError:
    XINPUT_AVAILABLE = False

_xinput_dll = None

# Button Masks
XINPUT_BUTTON_DPAD_UP    = 0x0001
XINPUT_BUTTON_DPAD_DOWN  = 0x0002
XINPUT_BUTTON_DPAD_LEFT  = 0x0004
XINPUT_BUTTON_DPAD_RIGHT = 0x0008
XINPUT_BUTTON_START      = 0x0010
XINPUT_BUTTON_BACK       = 0x0020
XINPUT_BUTTON_RIGHT_THUMB = 0x0080
XINPUT_BUTTON_LEFT_SHOULDER  = 0x0100
XINPUT_BUTTON_RIGHT_SHOULDER = 0x0200
XINPUT_BUTTON_A          = 0x1000
XINPUT_BUTTON_B          = 0x2000
XINPUT_BUTTON_X          = 0x4000

def load_xinput():
    global _xinput_dll
    if not XINPUT_AVAILABLE or sys.platform != "win32": return False
    for name in ("xinput1_4", "xinput1_3", "xinput9_1_0"):
        try:
            _xinput_dll = ctypes.windll.LoadLibrary(name)
            return True
        except OSError: continue
    return False

if XINPUT_AVAILABLE:
    class XINPUT_GAMEPAD(ctypes.Structure):
        _fields_ = [
            ("wButtons",      ctypes.c_ushort),
            ("bLeftTrigger",  ctypes.c_ubyte),
            ("bRightTrigger", ctypes.c_ubyte),
            ("sThumbLX",      ctypes.c_short),
            ("sThumbLY",      ctypes.c_short),
            ("sThumbRX",      ctypes.c_short),
            ("sThumbRY",      ctypes.c_short),
        ]
    class XINPUT_STATE(ctypes.Structure):
        _fields_ = [("dwPacketNumber", ctypes.c_ulong), ("Gamepad", XINPUT_GAMEPAD)]
else:
    class XINPUT_GAMEPAD: pass
    class XINPUT_STATE: pass

def get_xinput_state(pad=0):
    if _xinput_dll is None: return None
    state = XINPUT_STATE()
    if _xinput_dll.XInputGetState(pad, ctypes.byref(state)) != 0: return None

    # Analog Triggers [0.0, 1.0]
    rt = state.Gamepad.bRightTrigger / 255.0
    lt = state.Gamepad.bLeftTrigger / 255.0

    # Analog Steering [-1.0, 1.0] with deadzone
    lx = state.Gamepad.sThumbLX
    steer = 0.0
    if abs(lx) > 8000:  
        steer = (lx - 8000) / 24768.0 if lx > 0 else (lx + 8000) / 24768.0

    btns = state.Gamepad.wButtons
    return rt, lt, steer, btns