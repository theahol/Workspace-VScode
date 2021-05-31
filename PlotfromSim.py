from opcua import ua, Server
import numpy as np
import json
import matplotlib.pyplot as plt


'''
def disconnect (self):
    client.close()

if __name__ == "__main__":

    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # setup our server
    server = Server()
    server.set_endpoint("opc.tcp://localhost:4840/motopcua/server/")

    # setup our own namespace, not really necessary but should as spec
    uri = "http://server.motopcua.github.io"
    idx = server.register_namespace(uri)

    # get Objects node, this is where we should put our nodes
    objects = server.get_objects_node()

    # populating our address space
    myobj = server.nodes.objects.add_object(idx, "Moto")
    myvar = myobj.add_variable(idx, "MyVariable", 6.7)

    myvar.set_writable()    # Set MyVariable to be writable by clients
    
    # Adding desired variables
    s = myobj.add_variable(idx, "s", 6.7)
    l = myobj.add_variable(idx, "l", 6.7)
    u = myobj.add_variable(idx, "u", 6.7)
    r = myobj.add_variable(idx, "r", 6.7)
    b = myobj.add_variable(idx, "b", 6.7)
    t = myobj.add_variable(idx, "t", 6.7)

    s.set_writable()
    l.set_writable()
    u.set_writable()
    r.set_writable()
    b.set_writable()
    t.set_writable()

    # starting!
    server.start()
    print("server is started")
    try:
        filename = "posFromSim.json"
        pos_vec = []

        while True:
            time.sleep(0.1)
            sim_s = s.get_value(position[0])
            sim_l = l.get_value(position[1])
            sim_u = u.get_value(position[2])
            sim_r = r.get_value(position[3])
            sim_b = b.get_value(position[4])
            sim_t = t.get_value(position[5])

            vec_deg = [sim_s, sim_l, sim_u, sim_r, sim_b, sim_t]
            vec_rad = np.deg2rad(vec_deg)
            #pos_vec.append(vec_rad)

            data = json.load(open(filename))
            data_vec = data["joint_values"]
            data_vec.append(vec_rad)
            data["joint_values"] = data_vec 
        
            with open(filename, "w") as f:
                json.dump(data, f, indent=2)
            
            pos_vec.append(vec_rad)
    
    finally:
        #close connection, remove subcsriptions, etc
        server.stop()
'''

filename1 = "joint_values_from_sim03.json"
filename2 = "joint_values_from_rob4.json"
#filename1 = "serverDouble_sin_test4.json"
#filename2 = "serverDouble_robot_test4.json"

def get_joint_values_from_file(filepath, variable_name):

    data = json.load(open(filepath))

    return np.asarray(data[variable_name])


TrajPoints_fromSim = get_joint_values_from_file(filepath="Thea/joint_values_from_sim03.json", variable_name="joint_values")
TrajPoints_fromSim_rad = np.deg2rad(TrajPoints_fromSim) #vec*np.pi/180

TrajPoints_fromRobot = get_joint_values_from_file(filepath="Thea/joint_values_from_rob4.json", variable_name="joint_values")

#print('size of robot file', shape(TrajPoints_fromRobot))
#print('Simulation---------------',TrajPoints_fromSim_rad)


rows = 3
cols = 2
index = 0
joint_names = ['S', 'L', 'U', 'R', 'B', 'T']
fig, ax = plt.subplots(rows,cols, figsize = (18,11), constrained_layout = True)

    
for row in range(rows):
    if not rows*cols == len(joint_names):
        break
    for col in range(cols):
        if index < rows * cols:
            ax[row, col].plot(TrajPoints_fromSim_rad[:,index], 'r', TrajPoints_fromRobot[:,index], 'b')
            ax[row, col].set_ylabel('angle (rad)')
            ax[row, col].set_xlabel('steps')
            ax[row, col].set_title('Joint {}'.format(joint_names[index]))
            ax[row, col].legend(('Sim', 'Rob'), loc = 'upper right')
            #ax[row, col].legend('Robot', loc = 'upper right')
 
            index += 1


plt.show()

'''
plt.plot(TrajPoints_fromSim.T)
plt.show()'''