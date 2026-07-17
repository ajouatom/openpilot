from openpilot.system.webrtc.webrtcd import main as webrtcd_main


def main() -> None:
  webrtcd_main(carrot_vision_mode=True)


if __name__ == "__main__":
  main()
