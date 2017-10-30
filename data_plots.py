import numpy as np
import matplotlib.pyplot as plt


def main():
    my_data = np.genfromtxt('./data/sensor_data_dump.csv', delimiter=',')
    xy, = plt.plot(my_data[:,6], my_data[:,7], 'y.', label='Mxy')
    xz, = plt.plot(my_data[:,6], my_data[:,8], 'r.', label='Mxz')
    yz, = plt.plot(my_data[:,7], my_data[:,8], 'c.', label='Myz')
    plt.legend(handles=[xy, xz, yz])
    plt.axis('equal')
    plt.show()

    
if __name__ == '__main__':
    main()