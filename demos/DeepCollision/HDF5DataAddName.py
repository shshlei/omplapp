import h5py
import argparse

def modify(hdf5, name):
    f = h5py.File(hdf5, "a")
    f.attrs['problem_name'] = name
    f.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='HDF5 Add Name.')
    parser.add_argument('--hdf5', default='h5_collision.hdf5',
                        help='Filename of the saved hdf5 file (default: h5_collision.hdf5).')
    parser.add_argument('--name', default='hybrid_ellipse_ellipse',
                        help='The problem name (default: hybrid_ellipse_ellipse).')
    args = parser.parse_args()
    modify(args.hdf5, args.name)
