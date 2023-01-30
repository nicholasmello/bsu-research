from BSUBaseControl import BSUBaseControl


def main():
    base = BSUBaseControl()
    base.move(1, 0.5)
    base.move(-1, -0.5)


if __name__ == "__main__":
    main()