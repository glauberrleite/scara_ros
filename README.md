![execute](https://github.com/user-attachments/assets/1956a9b0-bcff-482f-a9df-b1e2fa4ee14e)
![ex2](https://github.com/user-attachments/assets/14a4e05d-c116-4f44-bdfa-ff9f9c69ea6d)


Esse projet o foi desenvolvido no Ubuntu 22.04, adaptações podem ser necessárias para outros sistemas operacionais.
## Dependências
### ROS 2 humble
<https://docs.ros.org/en/humble/Installation.html>
### CoppeliaSim
<https://www.coppeliarobotics.com/>
### ZeroMQ remote API
```shell script
python3 -m pip install coppeliasim-zmqremoteapi-client
```
### Demais dependências
Você precisa de opencv >= 4.9.0
```shell script
pip install opencv-python
pip install transforms3d
sudo apt-get install ros-<ros_distro>-tf-transformations
```
## Executando
Obtendo o projeto
```shell script
git clone https://github.com/glauberrleite/scara_ros.git
cd scara_ros
```
Monte as interfaces que foram criadas para eese projeto.
```shell script
colcon build --packages-select custom_interfaces
```
Adicione o comando abaixo ao arquivo .bashrc
```shell script
source /path/to/project/install/setup.bash
```
Execute
```shell script
ros2 interface list | grep custom
```
A saída deve ser
-    custom_interfaces/srv/ArucoDetect
-    custom_interfaces/srv/GrabOrRelease
-    custom_interfaces/action/ScaraControl
-    custom_interfaces/action/VelControl
  
Em seguida, monte os demais pacotes
```shell script
colcon build 
```
Cada um dos comandos abaixo devem ser uma aba própria no terminal.

Node responsável por processar imagem fornecidas pelas simulação.
```shell script
ros2 run vision vision
```
responsável por controlar a velocidade da esteira
```shell script
ros2 run conveyor_wrapper  conveyor_bringup 
```
responsável por controlar o scara e o processo de agarrar e soltar
```shell script
ros2 run scara_supervisor run_supervisor
```
No CoppeliaSim, abra a scene _scara_conveyor.ttt_ e aperte o botão play para iniciar a simulação.

Observação: evite interromper a simulação durante a execução, não sendo possível reinicie o Node scara_supervisor

## Demonstração

<https://www.youtube.com/watch?v=fP8PIGcADGY&list=PLxz-dAsWolBij_SNEZkncT7CNBxZEUDhJ&index=1>
