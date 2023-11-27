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
