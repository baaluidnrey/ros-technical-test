# ros-technical-test

## Objectifs du test technique

- programmation Python en orienté objet;
- utilisation de ROS;
- utilisation de Docker.

L'archive en question est un TP que font les étudiants de Polytech dans le module de robotique expérimentale. Ce TP est particulier concerne la modélisation d'un manipulateur à 6ddl, avec poignet à axes concourrants. Lors de ce TP, ils font :

1. le paramétrage géométrique du robot avec la convention de Denavit-Hartenberg modifiée (Khalil-Kleinfinger);
2. implémentation des modèle géométriques directs et inverses et du modèle cinématique;
3. contrôle du robot avec les outils de ROS;
4. génération de trajectoire et interfaçage avec ROS.

Ce TP est très long mais puisqu'il contient certains éléments importants pour le poste (Docker, ROS, Python), on peut le prendre comme base de test technique. On s'intéressera uniquement à :

1. implémentation du modèle géométrique direct;
2. interfaçage avec ROS;
3. utilisation de Docker.

## Utilisation du conteneur Docker

1. Si ce n'est pas fait, installer docker : 

    1. Docker Engine (surtout pas Docker Desktop) : https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository
    2. Puis la configuration : https://docs.docker.com/engine/install/linux-postinstall/

2. Cloner le répertoire
```bash
$ cd $ROS_TEST_DIR
$ git clone https://gitlab.isir.upmc.fr/eurobin/ros-technical-test.git
```

3. Compiler le conteneur
```bash
$ cd $ROS_TEST_DIR\ros-technical-test
$ docker build --tag tp_fanuc_docker .
```

4. Lancer le conteneur
```bash
$ cd $ROS_TEST_DIR\ros-technical-test\docker
$ ./start_docker.zsh
```

Puis accéder au conteneur depuis d'autres terminaux.
```bash
$ docker exec -it tp_fanuc zsh
```

5. Une fois dans le docker, utilisation du TP. On arrive directement dans `catkin_ws` :

On compile, on source, et le reste est dans le sujet de TP. Mais on peut déjà tester le `launchfile` général.
```bash
$ catkin_make    # compiler
$ chmod +x src/robotique_experimentale/tp_fanuc/scripts/* # rendre les scripts exécutables
$ source devel/setup.zsh
$ roslaunch tp_fanuc tp_DHm.launch
```

## Ce qu'il faut coder

1. Coder les méthodes manquantes dans le modèle géométrique direct (`mgd.py`) : `compute_Ti(self,dh,q):`, `compute_T(self,Q,i,j)` et `compute_robot_state(self,Q)`.

La matrice de transformation homogène obtenue à partir des paramètres de Denavit-Hartenberg est la suivante :

$$
^{i-1}T_{i} = 
\left[ \begin{array}{ccc|c}
\cos\theta_i 	& -\sin\theta_i 	& 0 & a_{i-1} \\
\cos\alpha_{i-1} \cdot \sin\theta_i & \cos\alpha_{i-1} \cdot \cos\theta_i & -\sin\alpha_{i-1} & -d \cdot \sin\alpha_{i-1} \\
\sin\alpha_{i-1} \cdot \sin\theta_i & \sin\alpha_{i-1} \cdot \cos\theta_i & \cos\alpha_{i-1} & d \cdot \cos\alpha_{i-1} \\ \hline
0 & 0 & 0 & 1
\end{array}\right]
$$

2. Coder le noeud `traj_arti` qui permet de faire une interpolation de trajectoire articulaire entre deux positions et qui publie les consignes sur le topic `/joints_state`.