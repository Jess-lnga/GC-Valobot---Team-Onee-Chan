# Tests de la FSM interactive

Depuis une invite developpeur Visual Studio, a la racine du projet :

```bat
cl /nologo /W4 /std:c11 /Itests/stubs /Febuild/test_interactive.exe /Fobuild/test_interactive.obj tests/test_interactive.c
build\test_interactive.exe
```

Le dossier `build` doit exister. Le test execute `interactive.c` avec des capteurs
et une arrivee en position simules, sans connexion au robot. Il couvre les quatre
directions, les 3 secondes continues, les gestes interrompus et diagonaux, les
mesures invalides, l'attente a l'horizontale, la premiere image avant la marche,
les gestes TOF brefs/prolonges, l'hysteresis et le rearmement.

## Validation sur le robot

Le programme demarre en `IDLE_MODE`, debout en attente. Dans `interactive.c` :

- Front : pitch < -30 deg, suivi de ligne.
- Back : pitch > 30 deg, attente reservee au futur suivi de balle.
- Left : roll < -30 deg, interactions TOF, tete levee.
- Right : roll > 30 deg, attente debout.

Maintenir une seule direction 3 secondes. Les pattes se replient puis se
recentrent. Reposer le robot : pitch et roll doivent rester sous 5 deg en valeur
absolue pendant 500 ms pour commencer a se relever. La selection reste possible
depuis chaque mode actif. Une inclinaison diagonale au-dela des deux seuils est
ambigue et annule le maintien. Les transitions elles-memes ne sont pas reselectables.

En mode Left, un seul TOF lateral doit detecter la main. Un geste bref provoque
une torsion puis un retour. Apres 1500 ms continus, les diagonales sont levees,
recentrees et reposees successivement pour conserver une rotation approximative
du corps. La main doit etre retiree des deux TOFs lateraux avant un nouveau geste.
Le TOF frontal commande independamment la tete pendant la presence de la main.

Verifier le sens de `LEFT_TURN_US` / `RIGHT_TURN_US` sur la mecanique. Ces valeurs
et l'amplitude de rotation sont a calibrer ; il n'y a pas de mesure de cap absolu.
Les seuils TOF (150 mm / 200 mm) utilisent les distances corrigees des offsets du
projet. Valider les trajectoires avec le chassis soutenu, puis au sol avec assez
d'espace. Les tests simules ne valident ni le couple, ni les collisions, ni le courant.

Les rampes et les gestes sont non bloquants. Le suivi de ligne reutilise les
phases de marche existantes : la selection IMU reprend entre ces phases, pas au
milieu d'une phase bloquante. Une valeur IMU absente suspend les actions et
interrompt le maintien de selection. Le retour a l'horizontale est une condition
de posture, pas une preuve de contact avec le sol.

Une fois les 500 ms a l'horizontale valides, le relevement est verrouille jusqu'a
son terme : les variations IMU provoquees par le mouvement ne relancent pas le
repli. Les tests couvrent ce cas, y compris une mesure absente pendant le relevement.

Les gestes TOF utilisent une rampe de pattes de 25 us par pas de 20 ms (contre
10 us pour les transitions de mode). La torsion vise 1200/1800 us au lieu de
1300/1700 us. La tete conserve sa rampe de 10 us, independante des pattes, entre
`SERVO_HEAD_UP_US` (1250 us, moins relevee qu'avant) et `SERVO_HEAD_LINE_US`
(1600 us, position du suivi de ligne). Ces deux positions sont partagees avec
les fonctions de tete existantes pour eviter des consignes contradictoires.
La torsion simple et son retour au neutre utilisent desormais 20 us par pas,
contre 25 us pour la rotation avec deplacement des pattes. La torsion initiale
d'un geste prolonge utilise aussi ces 20 us, puisque le geste commence de la meme facon.

## Detection statistique de la balle rouge

Depuis une invite developpeur Visual Studio :

```bat
cl /nologo /W4 /std:c11 /Febuild/test_find_ball.exe /Fobuild/test_find_ball.obj tests/test_find_ball.c
build\test_find_ball.exe
```

Le test utilise directement la vraie implementation. Il couvre une tache rouge
avec du bruit eloigne, la moyenne et l'ecart-type, le cadre et la preservation du
reste de l'image, les couleurs non rouges, le rouge sombre, le seuil exact de
presence, le rejet apres filtrage statistique, les bords et les arguments invalides.

`find_ball(frame, width, height)` attend une image RGB565 fraiche avant les autres
filtres destructifs. Les coordonnees du resultat sont celles du buffer brut.
L'affichage Python effectue une rotation de 90 degres : un centre brut (x,y)
devient (height - 1 - y, x) dans cet affichage. Le cadre tourne avec l'image.
La detection n'est pas branchee automatiquement au mode Back ; ce module ne
commande aucun mouvement du robot.

Les seuils dans `frame_find_ball.h` sont des valeurs initiales. Les tests ne
prouvent pas leur adequation a tous les eclairages reels ni les FPS sur le Pico.
La methode suppose une seule cible rouge dominante : moyenne/ecart-type seuls
ne permettent pas de separer deux objets rouges, ni de rejeter tout bruit diffus
s'il contient assez de pixels rouges. Aucun test de circularite n'est impose.
