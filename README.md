# LABORATORIO-5-CINEMATICA-INVERSA
# Robinson Jair Orduz Gomez
# introducción:
en este laboratorio se hace cinematica inversa de un manipulador phantom x pincher, primero se tomaron las dimensiones de longitud de los eslabones y a partir de estas, con el algoritmo Denavit-Hartenberg se obtuvo el modelo matematico que permite saber el punto en el espacio en el que se encuentra el tcp del manipulador a partir de los angulos de las articulaciones y longitud de los eslabones, luego con las ecuaciones de cinematica inversa, se puede saber que angulo entre los eslabones es necesario para ubicar el TCP en un punto deseado.
# Método DH:
https://github.com/robinsonorduz/LABORATORIO-4-CINEMATICA-DIRECTA/blob/main/Captura.JPG
# Metodologia:
## instalar catkin build:
Es un complemento de ROS que permite comunicarlo con los motores Dinamixel del manipulador.
## instalar el paquete pxROBOT:
Este paquete crea los nodos para acceder a los servicios de los motores dinamixel
## instalar libreria de Peter Corke:
se instala para pyton, y permite dibujar las posiciones articulares del manipulador en una grafica que se ve en el PC.
# Cinematica inversa:
https://github.com/robinsonorduz/lab5-cinematica-inversa/blob/main/cin.%20inversa.png

A partir de relaciones trigonometricas, se obtienen las ecuaciones que dan los angulos de articulacion a partir de las coordenadas del TCP y la longitud de los eslabones:
```python
  xaux=np.sqrt(x**2+y**2)-0.110
    costheta3=(xaux**2+zaux**2-l1**2-l2**2)/(2*l1*l2)
    sentheta3=np.sqrt(1-costheta3**2)
    theta3=np.arctan2(sentheta3, costheta3)
theta2=(np.arctan2(zaux,xaux)+np.arctan2(l2*sentheta3, l1+l2*costheta3))
theta4=(-theta2+theta3)
```
se pone este algoritmo en una funcion que recibe las coordenadas del punto del espacio donde se quiere poner el TCP y retorna los angulos de las articulaciones:
```python
  def cinemInversa(x,y,z):
    l1=0.105
    l2=0.105
    xaux=np.sqrt(x**2+y**2)-0.110
    zaux=-0.063+z
    costheta3=(xaux**2+zaux**2-l1**2-l2**2)/(2*l1*l2)
    print(costheta3)
    sentheta3=np.sqrt(1-costheta3**2)
    theta3=np.arctan2(sentheta3, costheta3)
    theta2=(np.arctan2(zaux,xaux)+np.arctan2(l2*sentheta3, l1+l2*costheta3))
    theta1=np.arctan2(y,x)
    theta4=(-theta2+theta3)
    angulos=[theta1, -(np.pi/2-theta2), -(theta3), (theta4), 0]

    return angulos
```
del laboratorio 4 se usa la funcion del publicador para enviar los angulos al controlador del robot
```python
  def enviarPosicion(puntos,flag):
    pub = rospy.Publisher('/joint_trajectory', JointTrajectory, queue_size=0)
    rospy.init_node('joint_publisher', anonymous=False)

    for i in range(len(puntos)):
        puntoActual=puntos[i]
        x=puntoActual[0]
        y=puntoActual[1]
        z=puntoActual[2]

        state = JointTrajectory()
        state.header.stamp = rospy.Time.now()
        state.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
        point = JointTrajectoryPoint()
        posActual=cinemInversa(x,y,z)
        if flag==True: posActual[4]=-2
        else: posActual[4]=0
        point.positions = posActual
        point.time_from_start = rospy.Duration(0.5)
        state.points.append(point)
        pub.publish(state)
        rospy.sleep(3)
```
se comprueba:
```python

  def enviarAngulos(angulos):
    pub = rospy.Publisher('/joint_trajectory', JointTrajectory, queue_size=0)
    rospy.init_node('joint_publisher', anonymous=False)
    state = JointTrajectory()
    state.header.stamp = rospy.Time.now()
    state.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
    point = JointTrajectoryPoint()
    point.positions = angulos
    point.time_from_start = rospy.Duration(0.5)
    state.points.append(point)
    pub.publish(state)
    rospy.sleep(3)

```
# Definir los puntos de la trayectoria que va a describir el robot:
Se tomaron las medidas de la base de madera del robot para saber el tamaño de los dibujos y se transladaron esas medidas a autocad para diseñar alli las trayectorias:

https://github.com/robinsonorduz/lab5-cinematica-inversa/commit/842c2661f8ed4560b009325df99fe2f44c27edab

D e este boceto se obtiene una tabla que contiene las coordenadas x, y,z ; la coordenada z depende de que parte de la trayectoria dibuja y cual no.
Luego se configura una posicion de home:

https://github.com/robinsonorduz/lab5-cinematica-inversa/blob/main/home.png


# Secuencia de agarre de herramienta:
 Se hizo un script para que la herramienta haga el ciclo de agarre de la herramienta y se unio al resto de la rutina:
 ```python
  def joint_publisher():
    
    posicionHome=[0, 0, -np.pi/2, 0, 0]
    enviarAngulos(posicionHome)
    puntos=[[0.020,-0.260,0.12],[0.020,-0.260,0.028]]
    enviarPosicion(puntos,False)
    puntos=[[0.020,-0.260,0.028],[0.020,-0.260,0.12]]
    enviarPosicion(puntos,True)
    posicionHome=[0, 0, -np.pi/2, 0, -2]
    enviarAngulos(posicionHome)
    
    while True:
        
        print("Ingrese la tarea que quiere que el robot haga: ")
        print("1. Trazar alcance máximo")
        print("2. Trazar alcance mínimo")
        print("3. Iniciales de los nombres")
        print("4. Figura")

        tarea=int(input())

        if tarea==1:
            puntos=[[0.047,-0.29,0.08],[0.047,-0.29,0.0],[0.013,0.2945,0.0],[0.013,0.2945,0.08]]
            enviarPosicion(puntos,True)
            enviarAngulos(posicionHome)

        elif tarea==2:
            puntos=[[-0.071,-0.164,0.06],[-0.071,-0.164,0.0],[0.0075,0.178,0.0],[0.0075,0.178,0.06]]
            enviarPosicion(puntos,True)
            enviarAngulos(posicionHome)
        elif tarea==3:
            puntos=[[0.048,-0.2645,0.06],[0.048,-0.2645,0.0],[0.023,-0.1955,0.0],[0.051,-0.1855,0.0],[0.064,-0.187,0.0],[0.071,-0.2065,0.0],[0.0595,-0.2175,0.0],[0.0345,-0.2265,0.0],[0.059, -0.2175,0.0],[0.090,-0.247,0.0],[0.090,-0.247,0.06],[0.102,-0.2425,0.06],[0.102,-0.2425,0.0],[0.0775,-0.176,0.0],[0.1015,-0.191,0.0],[0.110,-0.164,0.0],[0.1335,-0.230,0.0],[0.1335,-0.230,0.06]]
            enviarPosicion(puntos,True)
            enviarAngulos(posicionHome)
       
        elif tarea==4:
            angulo=0
            puntos=[]
            puntos.append([0.068+0.04*np.cos(np.deg2rad(24)),0.227+0.04*np.sin(np.deg2rad(24)),0.10])

            for o in range(0,27):

                angulo=angulo+360/26
                radio=0.04+0.01*np.sin(4*np.deg2rad(angulo))
                puntos.append([0.068+radio*np.cos(np.deg2rad(angulo)),0.227+radio*np.sin(np.deg2rad(angulo)),0.0])
            puntos.append([0.068+radio*np.cos(np.deg2rad(o)),0.227+radio*np.sin(np.deg2rad(o)),0.10])
            enviarPosicion(puntos,True)
            enviarAngulos(posicionHome)
        else : 
            enviarAngulos(posicionHome)


            puntos=[[0.02,-0.25,0.12],[0.02,-0.25,0.025]]
            enviarPosicion(puntos,True)
            puntos=[[0.02,-0.25,0.025],[0.02,-0.25,0.12]]
            enviarPosicion(puntos,False)
            posicionHome=[0, 0, -np.pi/2, 0, -2]
            enviarAngulos(posicionHome)

            posicionHome=[0, 0, -np.pi/2, 0, 0]
            enviarAngulos(posicionHome)
            break

```
 # conclusiones:
 -por la construccion del robot, el robot describe la trayactoria que se le pide con un error visualmente notorio, que podria disminuirse dando un mayor numero de puntos o mejorando la construccion del robot.
