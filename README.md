# Welcome to *ensta-rob201*

This git repository *ensta-rob201* is the starting point for your work during the ROB201 project/course. The online, up-to-date version of this code is available at: [GitHub repository *ensta-rob201*](https://github.com/emmanuel-battesti/ensta-rob201)

To start working on your ROB201 project, you will have to install this repository (which will install required dependencies). The [INSTALL.md](INSTALL.md) file contains installation tips for Ubuntu and Windows. The code will work also on MacOS, with minimal adaptation of the Ubuntu installation instructions.

# Place-Bot

*ensta-rob201* use the **Place-Bot** simulator: [**Place-Bot** GitHub repository](https://github.com/emmanuel-battesti/place-bot). It will be installed automatically with the above procedure, but it is strongly recommended to read the [*Place-Bot* documentation](https://github.com/emmanuel-battesti/place-bot#readme).


# Submission

At the end of the course, you will have to submit your solution to your teacher in a zip file containing this complete repository. 

# Contact

If you have questions about the code, in particular the use of *Place-Bot* you can contact: emmanuel . battesti at ensta-paris . fr

# Notes perso

J'ai décidé d'installer l'environnement avec Conda, mais certains packages ont été installés avec pip (place-bot et ses dépendances). Ce n'est donc pas très propre. J'utilise python 3.11, je crois que ça marche. Au besoin je me dirigerais vers python 3.10 ou moins.

# Séance 1 : Installation et prise en main

Rien de spécila à préciser

Je n'ai pas fait la partie "extensions possibles" , peut-être à creuser.

# Séance 2 : Navigation réactive

TODO : metrre les paramètres dans la fonction de `my_robot_slam.py` plutot que dans la fonction de `control.py`


j'ai l'impression que c'et bugué genre le robot vas dans la direction opposée et fait pas un 360 quand le k_omega ets trop faible
update :j'avais oublié de remttre un angle dans l'intervall -pi pi
okay la c'est bon ! ça marche bien !

je nai pas fait la partie facultative

# Séance 3 : mappage slam

J'ai eu pleins de problèpe entre la V1 et la V2 à cause de np.clip ! quand on applique la méthode .clip, ça ne modifie pas l'array....
Il faut faire array = array.clip(min,max)
Actuellement je crois que le robot va un peu trop vite.

# Séance 4 : Localisation

# Séance 5 : Planification de trajectoire

D'abord, pas de cours magistral, on continue à travailler sur la séance 4

Objectif : avoir un slam et une localisation complètement fonctionnelle à la fin de la semaine

faire un profiling

si best score trop faible, on fait quand même un update de la map ? le problème que j'ai c'est que au début il y a pas grand chose sur la map car il ya peut d'itérations, et si je fais pas de update map bah le score reste toujours faible et donc c'est une catastripje

résultat du profiling : ce qui prend le plsu de temps c'est le add values along lines....   

le prof a dit que pour best score avec le seuil, il faut genre forcer la mise à jour de la map pour un certain nombre de premières itérations (le temps que la map soit "complète" genre que les valeurs au niveau des murs soient au max, 50 dans notre cas)

l'utilisation de la version CEM n'améliore pas vraiment les choses et augmente le temps de calcul
Ensuite je commence la "vrai" séance 5 : Planification de trajectoire