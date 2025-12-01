import serial
import struct
import numpy as np
import cv2
import time

PORT = "COM4"   # adapte si besoin
BAUD = 115200   # ignoré par l’USB CDC, mais nécessaire pour pyserial

MAGIC_START = b"OVF0"

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
    """Cherche la séquence magic dans le flux série."""
    buf = b""
    while True:
        b = ser.read(1)
        if not b:
            raise IOError("Timeout en attente de la magic de début")
        buf += b
        if buf.endswith(magic):
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

def main():
    ser = serial.Serial(PORT, BAUD, timeout=2)
    print(f"Ouvert sur {PORT}")
    ser.reset_input_buffer()

    last_time = time.time()
    frame_count = 0
    first_frame = True

    try:
        while True:
            try:
                # 1) Sync sur le début de frame
                sync_magic(ser, MAGIC_START)

                # 2) Lire le reste du header :
                #    version (1), width (2), height (2), payload_len (4), checksum (2)
                header = read_exact(ser, 11)
                version, width, height, payload_len, checksum_header = struct.unpack("<BHHIH", header)

                # Vérif basique payload_len
                if payload_len != width * height * 2:
                    print(f"[WARN] payload_len incohérent : {payload_len} vs {width*height*2}")
                    continue

                # 3) Lire les données RGB565
                raw = read_exact(ser, payload_len)

                # 4) Vérifier checksum
                frame565 = np.frombuffer(raw, dtype="<u2")  # uint16 little-endian
                calc_checksum = int(frame565.sum() & 0xFFFF)

                if checksum_header != calc_checksum:
                    print(f"[WARN] Checksum mismatch: header={checksum_header} calc={calc_checksum}")
                    continue

                # 5) Reshape & conversion en BGR
                try:
                    frame565 = frame565.reshape((height, width))
                except ValueError:
                    print("[WARN] reshape impossible, resync...")
                    continue

                frame_bgr = rgb565_to_bgr(frame565)

                # 6) Affichage + FPS
                frame_count += 1
                now = time.time()
                dt = now - last_time
                fps = None
                if dt >= 1.0:
                    fps = frame_count / dt
                    frame_count = 0
                    last_time = now

                if first_frame:
                    print(f"Première frame OK : {width}x{height}")
                    first_frame = False

                display = frame_bgr.copy()
                if fps is not None:
                    cv2.putText(
                        display,
                        f"{width}x{height}  FPS: {fps:.1f}",
                        (5, 15),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (255, 255, 255),
                        1,
                        cv2.LINE_AA,
                    )

                cv2.imshow("OV7670 (Pico)", display)
                key = cv2.waitKey(1) & 0xFF
                if key == 27:  # ESC
                    break

            except IOError as e:
                print(f"[ERREUR] {e}, tentative de resync...")
                continue

    finally:
        ser.close()
        cv2.destroyAllWindows()
        print("Fermé.")

if __name__ == "__main__":
    main()
