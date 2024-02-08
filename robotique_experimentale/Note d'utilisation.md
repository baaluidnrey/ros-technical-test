# Note d'utilisation

1. Installer la version `desktop-full` de `ROS noetic` : http://wiki.ros.org/noetic/Installation


2. Copier le répertoire `robotique_experimentale` dans le répertoire de travail `ROS`, usuellement `~/catkin_ws/src`


3. Compiler

```bash
cd ~/catkin_ws
catkin_make
```

4. Exécuter :

Afin d'exécuter le code, taper les instructions suivantes dans un ou des terminaux.
Il est possible d'utiliser le logiciel `terminator` pour pouvoir séparer son terminal.
Chaque terminal doit avoir les variables d'environnement de `ROS` (`source /opt/ros/noetic/setup.bash` ou l'inclure directement dans le `bashrc` pour ne pas avoir à le faire à chaque fois.)

Pour tout exécuter, utiliser :
```bash
roslaunch tp_fanuc tp_DHm.launch
```

Les scripts ont des arguments permettant de tester le programmme. Les différents cas d'usage sont précisés ci-dessous.

- Modèle géométrique direct
  - Test
    - avec le robot test
      ```bash
      # charger la configuration du robot (dans un terminal)
      roslaunch tp_fanuc load_config.launch

      # executer le noeud du modele geometrique direct en mode test avec le robot test (dans un autre terminal)
      rosrun tp_fanuc mgd.py --test --robot=robot_test
      ```
    - avec le *Fanuc LR-Mate*
      ```bash
      # charger la configuration du robot (dans un terminal)
      roslaunch tp_fanuc load_config.launch

      # executer le noeud du modele geometrique direct en mode test (dans un autre terminal)
      rosrun tp_fanuc mgd.py --test
      ```
  - Utilisation avec animation sous Rviz
    ```bash
    # execution du modele geometrique direct uniquement
    roslaunch tp_fanuc tp_DHm.launch mgi:=false jacobienne:=false
    ```

- Modèle géométrique inverse
  - Test
    ```bash
    # charger la configuration du robot (dans un terminal)
    roslaunch tp_fanuc load_config.launch

    # executer le noeud du modele geometrique direct en mode test (dans un autre terminal)
    rosrun tp_fanuc mgd.py --test

    # executer le noeud du modele geometrique inverse en mode test (encore dans un autre terminal)
    rosrun tp_fanuc mgi.py --test
    ```
  - Utilisation avec animation sous Rviz
    ```bash
    # execution des modeles geometriques direct et inverse uniquement
    roslaunch tp_fanuc tp_DHm.launch jacobienne:=false
    ```

- Modèle cinématique
  - Test
    - avec le robot test
    ```bash
    # charger la configuration du robot (dans un terminal)
    roslaunch tp_fanuc load_config.launch

    # executer le noeud du modele geometrique direct en mode test avec le robot test (dans un autre terminal)
    rosrun tp_fanuc mgd.py --test --robot=robot_test

    # executer le noeud du modele cinematique en mode test avec un robot test (encore dans un autre terminal)
    rosrun tp_fanuc jacobienne.py --test --robot=robot_test
    ```
    - avec le *Fanuc LR Mate*
    ```bash
    # charger la configuration du robot (dans un terminal)
    roslaunch tp_fanuc load_config.launch

    # executer le noeud du modele geometrique direct en mode test (dans un autre terminal)
    rosrun tp_fanuc mgd.py --test

    # executer le noeud du modele cinematique en mode test (encore dans un autre terminal)
    rosrun tp_fanuc jacobienne.py --test
    ```
  - Utilisation avec animation sous Rviz
    ```bash
    roslaunch tp_fanuc tp_DHm.launch
    ```


- Exemples de vecteurs pour les tests

  Faire `Ctrl+Maj+V` pour coller dans le terminal

  - robot test
    - 0.0 0.0
    - 0.0 1.0
    - 1.571 1.0
  - *Fanuc LR Mate*
    - 0.0 0.0 0.0 0.0 0.0 0.0
    - 1.571 0.0 0.0 0.0 0.0 0.0
