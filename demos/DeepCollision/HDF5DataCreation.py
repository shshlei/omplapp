import h5py
import numpy as np
import argparse

def create(data, hdf5, objects, points, length):
    f = h5py.File(hdf5, "w")
    f.attrs["objects"] = objects
    f.attrs["points"] = points
    f.attrs["labels"] = objects * points
    datas = np.loadtxt(data)
    for i in range(objects):
        g = f.create_group("scenario_{:n}".format(i))
        g.create_dataset("envs", data=datas[i*points, 0:length])
        g.create_dataset("robots", data=datas[i*points:i*points+points, length:-1])
        g.create_dataset("labels", data=datas[i*points:i*points+points, -1])
    f.close()

def read(hdf5):
    f = h5py.File(hdf5, "r")

    # Print the keys of groups and datasets under '/'.
    print(f.filename, ":")
    print(f.attrs["objects"])
    print(f.attrs["points"])
    print(f.attrs["labels"])
    print([key for key in f.keys()], "\n")

    g = f["scenario_0"]
    print([key for key in g.keys()])
    print(g['envs'][:])		# 3. relative path: group[]
    print(g['robots'][:])		# 3. relative path: group[]
    print(g['labels'][:])		# 3. relative path: group[]

    # Save and exit the file
    f.close()

def modify(hdf5):
    f = h5py.File(hdf5, "a")
    f.attrs['problem_name'] = 'hybrid_ellipse_ellipse'
    # f.attrs['points'] = f.attrs['Points']
    # del f.attrs['Points']
    # Save and exit the file
    f.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='HDF5 Data Creation.')
    parser.add_argument('--data', default=None,
                        help='Filename of the collision data.')
    parser.add_argument('--hdf5', default='h5_collision.hdf5',
                        help='Filename of the saved hdf5 file (default: h5_collision.hdf5).')
    parser.add_argument('--objects', type=int, default=1000, metavar='N',
                        help='Objects number (default: 1000)')
    parser.add_argument('--points', type=int, default=1000, metavar='N',
                        help='Points number (default: 1000)')
    parser.add_argument('--length', type=int, default=6, metavar='N',
                        help='length of the each data (default: 6)')
    args = parser.parse_args()

    #create(args.data, args.hdf5, args.objects, args.points, args.length)
    #read(args.hdf5)
    modify(args.hdf5)
