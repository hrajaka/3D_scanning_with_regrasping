import trimesh
import sys


if __name__ == '__main__':
    if len(sys.argv != 2):
        print("Please provide a filename")
        sys.exit()

    pawn = trimesh.load(sys.argv[1])
    pawn.show()
