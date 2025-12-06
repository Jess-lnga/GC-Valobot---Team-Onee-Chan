import serial
import struct
import threading
import time

import numpy as np
import cv2

# ---------------------------------------------------------------------
# Config port série
# ---------------------------------------------------------------------
PORT = "COM4"   # à adapter
BAUD = 115200   # ignoré par l’USB CDC mais requis par pyserial

MAGIC_START = b"OVF0"

ENABLE_SHARPEN = True
ENABLE_SATURATION_BOOST = True
SATURATION_FACTOR = 1.3
SHARPEN_AMOUNT = 1.5

# Taille initiale de la fenêtre
INITIAL_WIN_W = 800
INITIAL_WIN_H = 600

# ---------------------------------------------------------------------
# Utilitaires
# ---------------------------------------------------------------------
def read_exact(ser, n):
    """Lit exactement n octets (ou lève une erreur)."""
    data = b""
    while len(data) < n:
        chunk = ser.read(n - len(data))
        if not chunk:
            raise IOError("Timeout ou déconnexion pendant la lecture")
        data += chunk
    return data


def sync_magic(ser, magic=MAGIC_START):
    """
    Cherche la séquence magic dans le flux série.
    NOTE: buffer tronqué pour ne pas grossir à l’infini.
    """
    buf = b""
    mlen = len(magic)
    while True:
        b = ser.read(1)
        if not b:
            raise IOError("Timeout en attente de la magic de début")
        buf += b
        if len(buf) > mlen:
            buf = buf[-mlen:]
        if buf == magic:
            return


def rgb565_to_bgr(frame565):
    """
    frame565 : array (H, W) uint16, RGB565
    Retourne une image BGR uint8 (pour OpenCV).
    """
    r = ((frame565 >> 11) & 0x1F).astype(np.uint16)
    g = ((frame565 >> 5) & 0x3F).astype(np.uint16)
    b = (frame565 & 0x1F).astype(np.uint16)

    r = (r * 255 // 31).astype(np.uint8)
    g = (g * 255 // 63).astype(np.uint8)
    b = (b * 255 // 31).astype(np.uint8)

    img_bgr = np.dstack((b, g, r))
    return img_bgr


def enhance_image(frame_bgr):
    """Sharpen + boost de saturation pour un rendu plus sympa."""
    img = frame_bgr

    if ENABLE_SHARPEN:
        blur = cv2.GaussianBlur(img, (0, 0), 1.0)
        img = cv2.addWeighted(img, SHARPEN_AMOUNT, blur,
                              -(SHARPEN_AMOUNT - 1.0), 0)

    if ENABLE_SATURATION_BOOST:
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
        s = np.clip(s.astype(np.float32) * SATURATION_FACTOR,
                    0, 255).astype(np.uint8)
        hsv = cv2.merge((h, s, v))
        img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

    return img


# ---------------------------------------------------------------------
# Thread de capture : lit en continu, stocke la dernière frame
# ---------------------------------------------------------------------
class CaptureThread(threading.Thread):
    def __init__(self, ser):
        super().__init__(daemon=True)
        self.ser = ser
        self.running = True

        self.lock = threading.Lock()
        self.latest_frame = None
        self.latest_size = None  # (width, height)
        self.fps = 0.0

        self._frame_count = 0
        self._last_time = time.time()

    def stop(self):
        self.running = False

    def get_latest_frame(self):
        """Retourne (frame_bgr, (w,h), fps) ou (None, None, 0.0)."""
        with self.lock:
            if self.latest_frame is None:
                return None, None, 0.0
            return (self.latest_frame.copy(),
                    self.latest_size,
                    self.fps)

    def run(self):
        try:
            while self.running:
                try:
                    # 1) Sync sur la magic
                    sync_magic(self.ser, MAGIC_START)

                    # 2) Header : version (1), width (2), height (2),
                    #             payload_len (4), checksum (2)
                    header = read_exact(self.ser, 11)
                    version, width, height, payload_len, checksum_header = \
                        struct.unpack("<BHHIH", header)

                    expected_payload = width * height * 2
                    if payload_len != expected_payload:
                        # Désync → on saute cette frame et on resync
                        print(f"[WARN] payload_len {payload_len} != {expected_payload}, resync")
                        continue

                    # 3) Données RGB565 brutes
                    raw = read_exact(self.ser, payload_len)

                    # 4) Checksum
                    frame565 = np.frombuffer(raw, dtype="<u2")  # little-endian
                    calc_checksum = int(frame565.sum() & 0xFFFF)
                    if checksum_header != calc_checksum:
                        print(f"[WARN] checksum mismatch: header={checksum_header} calc={calc_checksum}")
                        continue

                    # 5) reshape + conversion
                    try:
                        frame565 = frame565.reshape((height, width))
                    except ValueError:
                        print("[WARN] reshape impossible, resync...")
                        continue

                    frame_bgr = rgb565_to_bgr(frame565)
                    frame_bgr = enhance_image(frame_bgr)

                    # 6) FPS
                    self._frame_count += 1
                    now = time.time()
                    dt = now - self._last_time
                    if dt >= 1.0:
                        self.fps = self._frame_count / dt
                        self._frame_count = 0
                        self._last_time = now

                    # 7) Stocker comme dernière frame
                    with self.lock:
                        self.latest_frame = frame_bgr
                        self.latest_size = (width, height)

                except IOError as e:
                    if not self.running:
                        break
                    print(f"[Capture] Erreur IO: {e} (resync...)")
                    continue

        finally:
            print("[Capture] Thread arrêté.")


# ---------------------------------------------------------------------
# Programme principal (UI)
# ---------------------------------------------------------------------
def main():
    ser = serial.Serial(PORT, BAUD, timeout=2)
    print(f"Ouvert sur {PORT}")
    ser.reset_input_buffer()

    cap_thread = CaptureThread(ser)
    cap_thread.start()

    window_name = "OV7670 (Pico)"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, INITIAL_WIN_W, INITIAL_WIN_H)

    last_frame = None
    last_size = None

    try:
        while True:
            # Récupérer la dernière frame disponible
            frame, size, fps = cap_thread.get_latest_frame()
            if frame is not None:
                last_frame = frame
                last_size = size

            # Si on a au moins une frame, on l’affiche
            if last_frame is not None:
                display = last_frame.copy()
                w, h = last_size

                # Overlay info
                txt = f"{w}x{h}"
                if fps > 0:
                    txt += f"  FPS: {fps:.1f}"
                cv2.putText(
                    display,
                    txt,
                    (5, 15),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 255),
                    1,
                    cv2.LINE_AA,
                )

                # Gestion du resize de la fenêtre
                try:
                    _, _, win_w, win_h = cv2.getWindowImageRect(window_name)
                    if win_w > 0 and win_h > 0:
                        display_resized = cv2.resize(
                            display,
                            (win_w, win_h),
                            interpolation=cv2.INTER_NEAREST
                        )
                    else:
                        display_resized = display
                except cv2.error:
                    display_resized = display

                cv2.imshow(window_name, display_resized)

            # Gestion des événements clavier / fermeture fenêtre
            key = cv2.waitKey(30) & 0xFF
            if key == 27:  # ESC
                break

            # Si la fenêtre est fermée à la croix
            if cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1:
                break

    finally:
        print("Fermeture en cours...")
        cap_thread.stop()
        cap_thread.join(timeout=2.0)
        ser.close()
        cv2.destroyAllWindows()
        print("Fermé proprement.")


if __name__ == "__main__":
    main()
