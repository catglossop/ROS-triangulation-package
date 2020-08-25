import numpy as np
import math
import matplotlib.pyplot as plt
import os, glob, cv2, csv, time
import sys, getopt
import pandas as pd



np_load_old = np.load
np.load = lambda *a,**k: np_load_old(*a, allow_pickle=True, **k)

#------------------------------------------------------------------------------#
#DRAW_BB: Requires input of two points: 1 at the top left corner of the object
#and 1 at the bottom right. The object centre for each image is computed and
#stored in order

#Arguments:
#directory: working directory for triangulation
#data: CSV containing triangulation information*
#points: numpy array name to save to
#samples: number of sample images being processed
#------------------------------------------------------------------------------#

def draw_bb(directory, data, points):
    with open(data,"r") as f:
        reader = csv.reader(f,delimiter = ",")
        dat = list(reader)
        samples = len(dat)
    vertices = np.zeros(shape=(samples,2)) # create an empty array to store points
    cnt = 0
    data_read = open(os.path.join(directory, data)) # open csv containing image location, transformation matrix, current location, camera matrix
    images = csv.reader(data_read)
    data_read.readline()
    for filename in images:
        print "Current File Being Processed is: " + filename[0]
        print "This is sample #" + str(cnt + 1)
        print '\n'
        img = cv2.imread(filename[0])
        plt.imshow(img) # open image for point inputs
        light_centre = plt.ginput(2)
        light_centre = get_centre(light_centre) # gets object centre point
        vertices[cnt] = light_centre
        cnt += 1
    data_read.close()
    np.save(os.path.join(directory,points), vertices) # saves numpy array
    return True

#------------------------------------------------------------------------------#
#GET_CENTRE: takes in two point input from DRAW_BB and computes centres

#Arguments:
#diag: two point array from DRAW_BB
#------------------------------------------------------------------------------#

def get_centre(diag):
    up_vertex = diag[0]
    down_vertex = diag[1]
    x = math.floor((down_vertex[0] - up_vertex[0])/2) + up_vertex[0]
    y = math.floor((down_vertex[1] - up_vertex[1])/2) + up_vertex[1]
    centre = np.array([[x,y]])
    return centre

#------------------------------------------------------------------------------#
#GET_JACOBIAN: gives the Jacobian for the general triangulation equations

#Arguments:
#matrix: camera matrix for sample being currently processed
#p_i: current guess at current iteration for 3D world coordinate location of object
#------------------------------------------------------------------------------#

def get_jacobian(matrix, p_i):
    P = matrix
    p_guess = p_i
    P = P.reshape(4,4) #reshapes into 4 x 4
    P_D = P[2][0]*p_guess[0,0] + P[2][1]*p_guess[1,0] + P[2][2]*p_guess[2,0] + P[2][3]*p_guess[3,0] #get the denominator of all rows
    P_N1 = P[0][0]*p_guess[0,0] + P[0][1]*p_guess[1,0] + P[0][2]*p_guess[2,0] + P[0][3]*p_guess[3,0] #get the numerator of the first row
    P_N2 = P[1][0]*p_guess[0,0] + P[1][1]*p_guess[1,0] + P[1][2]*p_guess[2,0] + P[1][3]*p_guess[3,0] #get the numerator of the second row
    row_1 = np.array([(P_D*P[0][0]-P_N1*P[2][0])/P_D**2, (P_D*P[0][1]-P_N1*P[2][1])/P_D**2, (P_D*P[0][2]-P_N1*P[2][2])/P_D**2, (P_D*P[0][3]-P_N1*P[2][3])/P_D**2]) #form the first row
    row_2 = np.array([(P_D*P[1][0]-P_N2*P[2][0])/P_D**2, (P_D*P[1][1]-P_N2*P[2][1])/P_D**2, (P_D*P[1][2]-P_N2*P[2][2])/P_D**2, (P_D*P[1][3]-P_N1*P[2][3])/P_D**2]) #form the second row
    J = np.matrix(row_1)
    J = np.append(J, [row_2], axis = 0) #form the 2x4 Jacobian matrix
    return J

#------------------------------------------------------------------------------#
#CONVERT_TO_MATRIX: converts camera matrix from a string from csv file into float matrix

#Arguments:
#mat_str: string containing 16 elements of the matrix
#------------------------------------------------------------------------------#

def convert_to_matrix(matrix_str):
    P = np.array([])
    for i in matrix_str.split(' '): #split at the spaces between elements
        if i != '': #if there is a empty string, remove it otherwise convert to a float and append
            P = np.append(P,float(i))
    return P

#------------------------------------------------------------------------------#
#SHOW_FINAL: shows the final world location projection overlaid on the labelled images

#Arguments:
#directory: working directory for trinagulation
#data: CSV containing triangulation information*
#points: nump array containing labelled object centres
#p_final: output of triangulation algorithm
#------------------------------------------------------------------------------#

def show_final(directory, data, points, p_final):
    points = np.load(os.path.join(directory, points)) #load points from draw_bb
    data = open(os.path.join(directory,data)) #open CSV
    samples = csv.reader(data)
    data.readline()
    cnt = 0
    p_final = p_final.reshape(4,1)
    for line in samples:
        P = convert_to_matrix(line[1])
        P = P.reshape(4,4)
        x = np.matmul(P,p_final)
        x = x/x[2]
        x = x[0:2]
        im = plt.imread(line[0])
        implot = plt.imshow(im)
        plt.scatter([x[0]], [x[1]])
        plt.scatter([points[cnt][0]], [points[cnt][1]], c='r')
        plt.show()
        cnt += 1
    return True

#------------------------------------------------------------------------------#
#COMBINE_CSV: takes in two csv files and combines them into a new single file

#Arguments:
#file1: first input file
#file2: second input file
#outputfile: output concatenated file
#------------------------------------------------------------------------------#

def combine_csv(file1, file2, outputfile):
    dir = os.getcwd()
    df = open(os.path.join(dir,file1))
    with open(outputfile , 'w') as new:
        new_writer = csv.writer(new)
        read = csv.reader(df)
        df.readline()
        for row in read:
            new_writer.writerow(row)
        df.close()
        new.close()
    data1 = open(os.path.join(dir, outputfile), 'a')
    csv_writer = csv.writer(data1)
    data2 = open(os.path.join(dir, file2))
    read = csv.reader(data2)
    data2.readline()
    for line in read:
        csv_writer.writerow(line)
    data2.close()
    data1.close()
    return True

#-----------------------------------------------------------------------------#
#LLS_INITIALIZE: initializes the triangulation algorithm by computing am initial
#guess with linear least squares

#Arguments:
#directory: working directory for triangulation
#data: CSV containing triangulation information*
#points: object centres outputed from DRAW_BB
#cam_intrin: camera intrinsics matrix in 3x3 form
#------------------------------------------------------------------------------#

def LLS_initialize(directory, data, points, cam_intrin):
    K = cam_intrin #load camera matrix
    points = np.load(os.path.join(directory, points)) #load points from draw_bb
    data = open(os.path.join(directory,data)) #open CSV
    samples = csv.reader(data)
    data.readline()
    N = np.zeros(shape = (3,3)) #initialize matrices for summing over all images
    N_c = np.zeros(shape = (3,1))
    cnt = 0
    for line in samples:
        pos = (convert_to_matrix(line[3])) #get odom frame position from rosbag
        pos_h = (np.append(pos, [1], axis=0)).reshape(4,1) #convert to homogenous form
        T = convert_to_matrix(line[2]) #convert string matrices from CSV to matrices
        T = T.reshape(4,4)
        [u,v] = points[cnt] #get object centre from draw_bb array
        T_inv_4 = np.linalg.inv(T) #get the 4x4 inverse of the transformation matrix
        T_inv = np.delete(T_inv_4,3,0) #get the 3x4 inverse of the transformation matrix
        K_inv = np.delete(np.linalg.inv(K),3,1) #get the 4x3 inverse of the intrinsics matrix
        P_3 = np.matmul(T_inv, K_inv) #get the resulting 3x3 camera matrix
        x_i = np.matrix([[u],[v],[1]]) #get the homogenous form of the object centre
        v_j = np.matmul(P_3, x_i) #compute the unit direction vector from the 2D pixel point to 3D world point
        v_j = v_j/np.linalg.norm(v_j)
        v_j_T = v_j.transpose() #get the transpose
        N_temp = np.identity(3) - np.matmul(v_j,v_j_T)
        N_c_temp = np.matmul(N_temp,np.delete(pos_h,3,0))
        N = N + N_temp #add matrices calculated for this image to sums
        N_c = N_c + N_c_temp
        cnt += 1
    data.close()
    p = np.matmul(np.linalg.inv(N),N_c) #get the estimate of the location of the object
    p_init = (np.asarray(np.vstack([p,[1]]))).reshape(-1) #reshape and turn into homogenous form
    print("3D WORLD LOCATION AT: " + str(p_init))
    return p_init

#------------------------------------------------------------------------------#
#TRIANGULATION: performs the iterative non-linear least squares portion of the
#triangulation with the intial guess from LLS_initialize

#Arguments:
#directory: working directory for triangulation
#data: CSV containing triangulation information*
#points: object centres outputed from DRAW_BB
#initial: output from LLS_INITIALIZE
#iterations: number of iterations
#lamb: value of lambda
#show_img: boolean show or don't show images as they are processed
#------------------------------------------------------------------------------#

def triangulation(directory, data, points, initial, iterations, lamb, show_img):
    y_residual = np.array([]) #initial vector for cost plot
    points = np.load(os.path.join(directory, points)) #load points from draw_bb
    delta_p = np.array([[0],[0],[0],[0]])  #initialize delta_p
    p_guess = initial.reshape(4,1)
    for iter in range(0,iterations): #start iterations
        data_read = open(os.path.join(directory,data)) #open CSV file
        samples = csv.reader(data_read)
        data_read.readline()
        res = 0 #initialize, total residual, A matrix, b matrix, cnt
        cnt = 0
        A = np.empty([4,4])
        b = np.empty([4,1])
        for row in samples:
            P = convert_to_matrix(row[1]) #get P matrix from CSV
            P = P.reshape(4,4)
            J = get_jacobian(P,p_guess) #get jacobian
            P_D = P[2][0]*p_guess[0,0] + P[2][1]*p_guess[1,0] + P[2][2]*p_guess[2,0] + P[2][3]*p_guess[3,0]
            P_N1 = P[0][0]*p_guess[0,0] + P[0][1]*p_guess[1,0] + P[0][2]*p_guess[2,0] + P[0][3]*p_guess[3,0]
            P_N2 = P[1][0]*p_guess[0,0] + P[1][1]*p_guess[1,0] + P[1][2]*p_guess[2,0] + P[1][3]*p_guess[3,0]
            X_i = np.matrix([[P_N1/P_D],[P_N2/P_D]]) #get predicted centre from p_guess
            if (show_img == True): #show overlay of predicted and labelled points on current image
                im = plt.imread(row[0])
                implot = plt.imshow(im)
                plt.scatter([X_i[0]], [X_i[1]])
                plt.scatter([points[cnt][0]], [points[cnt][1]], c='r')
                plt.show()
            res_i = X_i - points[cnt].reshape(2,1) #calculate residual between current point set
            res_temp = np.linalg.norm(res_i)**2 #compute squared residual error
            res = res + res_temp #add cost to total cost
            J_T = J.transpose()
            print(J)
            print(J_T)
            A_temp = np.matmul(J_T, J) #compute A matrix for current image
            A = A + A_temp  #sum to total
            b_temp = np.matmul(J_T, res_i) #compute b matrix for current image
            b = b + b_temp #sum to total
            cnt += 1
        data_read.close()
        M = A + lamb*(np.identity(4)*np.diagonal(A))
        print("This is M: ")
        print(M)
        print("This is b: ")
        print(b)
        delta_p = np.linalg.solve(M, b) #compute delta p for this iteration
        print("This is delta p: " + str(delta_p))
        p_guess = p_guess + delta_p #update p_guess
        p_guess = p_guess/p_guess[3,0] #normalize by the homogenous component
        print(p_guess)
        y_residual = np.append(y_residual,res) #add total cost to array for plotting
    x = np.linspace(0,len(y_residual),num=len(y_residual))
    plt.plot(x,y_residual,'r') #Cost vs. Iterations plot
    plt.title("Cost vs. Iterations")
    plt.xlabel("Number of iterations")
    plt.ylabel("Squared residual error")
    print("This is the final location: ")
    print(p_guess)
    plt.show()
    return p_guess

# PARAMETERS # ----------------------------------------------------------------#
input_dir = '/home/autoronto/triangulation'
iterations = 10
lamb = 0.0
samples_1 = 22
samples_2 = 0
samples_final = samples_1 + samples_2
data_final = 'data_trig_ped1.csv'
points_final = 'cam1_points.npy'
show_img = True
K = np.matrix([[1449.144043, 0, 1199.158041, 0],[0, 1456.590741, 1036.123288, 0],[0, 0, 1, 0], [0, 0, 0, 1]])

#init = np.array([277622,4686645,240,1])

# GROUND TRUTH ----------------------------------------------------------------#

#ped 1 right in front: (X: 277612.87 Y: 4686661.54 Z: 242.56 W: 1)
#ped 1 right right: (X: 277619.72 Y: 4686648.66 Z: 242.56 W: 1)

# EXECUTE ---------------------------------------------------------------------#

def main(argv):
    bb = False
    input_dir = ''
    data_final = ''
    show_img = False
    file1 = ''
    file2 = ''
    outputfile = ''
    try:
        opts, args = getopt.getopt(argv,"hb:d:c:s:m1:2:o:", ["box=","dir=", "csv=", "show=", "combine", "file1", "file2", "out"])
    except getopt.GetoptError:
        print 'Usage is: python triangulation.py --box <boolgetboundingboxes> --dir <workingdirectory> --csv <triangulationcsv> --show <boolshowimg>'
        print 'Usage is: python triangulation.py --combine -1 <file1> -2 <file2> -o <outputfile>'
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print 'Usage is: python triangulation.py --box <boolgetboundingboxes> --dir <workingdirectory> --csv <triangulationcsv> --show <boolshowimg>'
            print 'Usage is: python triangulation.py --combine -1 <file1> -2 <file2> -o <outputfile>'
            sys.exit()
        elif opt in ("-b", "--box"):
            bb = arg
        elif opt in ("-d", "--dir"):
            input_dir = arg
        elif opt in ("-c", "--csv"):
            data_final = arg
        elif opt in ("-s", "--show"):
            show_img = arg
        elif opt in ("-m", "--combine"):
            for opt, arg in opts:
                if opt in ('-1', 'file1'):
                    file1 = arg
                elif opt in ('-2', 'file2'):
                    file2 = arg
                elif opt in ('-o', "--out"):
                    outputfile = arg
            combine_csv(file1,file2,outputfile)
            print 'Successfully combined csv files'
            sys.exit()
    print "---------------------------------------------------------------"
    print "Working Directory is: ", input_dir
    print "Drawing bounding boxes is set to: ", bb
    print "Input CSV file is: ", data_final
    print "Show image is set to: ", show_img
    print '\n'
    if bb == 'True':
        draw_bb(input_dir, data_final, points_final)
        init = LLS_initialize(input_dir, data_final, points_final, K)
    if show_img == 'True':
        show_final(input_dir, data_final, points_final, init)

if __name__ == "__main__":
   main(sys.argv[1:])

np.load = np_load_old
