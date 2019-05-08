#!/home/cc/ee106b/sp19/class/ee106b-aai/virtualenvironment/my_new_app/local/bin/python
#!/home/cc/ee106b/sp19/class/ee106b-abj/python-virtual-environments/env/bin/python
import trimesh
import sys


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Please provide a filename")
        sys.exit()

    print('Visualizing {}'.format(sys.argv[1]))
    pawn = trimesh.load(sys.argv[1])
    pawn.fix_normals()
    pawn.show()
