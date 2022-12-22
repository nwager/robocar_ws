from game_controller import GameController
import sys

def main():
    controller = GameController(
        "Xbox Wireless Controller",
        "xbox_code_map.yaml"
    )

    while True:
        while True:
            read_str = ["{:.2f}".format(v) for v in controller.read()]
            sys.stdout.write("\r{}{}".format(read_str, " "*20))
            sys.stdout.flush()

if __name__ == "__main__":
    main()