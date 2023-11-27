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
- Por cinemática directa:
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
