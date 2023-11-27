# lab5-cinematica-inversa
# LABORATORIO-4-CINEMATICA-DIRECTA
# Robinson Jair Orduz Gomez
# introducción:
en este laboratorio se hace cinematica directa de un manipulador phantom x pincher, primero se tomaron las dimensiones de longitud de los eslabones y a partir de estas, con el algoritmo Denavit-Hartenberg se obtuvo el modelo matematico que permite saber el punto en el espacio en el que se encuentra el tcp del manipulador a partir de los angulos de las articulaciones y longitud de los eslabones.
# Método DH:
https://github.com/robinsonorduz/LABORATORIO-4-CINEMATICA-DIRECTA/blob/main/Captura.JPG
# Metodologia:
## instalar catkin build:
Es un complemento de ROS que permite comunicarlo con los motores Dinamixel del manipulador.
## instalar el paquete pxROBOT:
Este paquete crea los nodos para acceder a los servicios de los motores dinamixel
## instalar libreria de Peter Corke:
se instala para pyton, y permite dibujar las posiciones articulares del manipulador en una grafica que se ve en el PC.
