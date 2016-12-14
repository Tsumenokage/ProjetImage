Projet de traitement d'image : Détections des buts

Membres de l'équipe : 
ALBOUYS Jérémy
DODELIN Hugo
ZACCHELLO Jean-Baptiste

Notice : 

*Compilation du programme :
	Exécuter le MakeFile présent dans le dossier
	Si le MakeFile ne fonctionne pas à cause de librairies non trouvées, exéctuer la ligne de commande suivante : 
	"qmake-qt4 postDetection.pro"
	Executer le MakeFile une nouelle fois

*Exécution du programme : 
	Le progamme se compile sous le nom : postDestection
	Lancer le programme avec la ligne de commande suivante : 
	"./postDetection Chemin/Vers/Dossier/Image option"
	Le premier paramètre est le chemin vers le dossier contenant les images à traiter au format PNG
	Le second paramètre peut prendre deux valeurs :
		"b" Affiche uniquement un benchmark du programme
		"s" Affiche tous les résultats otenus lors des différents traitements

Données de tests :
        Nous avons utilisé log3 pour le développement et log2 pour les tests.

Exemple : ./postDetection ./images/log1/ b
