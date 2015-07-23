# arduino_cuve
Capteur de niveau par ultrason (SR04) pour une cuve, avec écran LCD 5110 envoi par RF (Format Oregon)
Une mesure du niveau de la cuve est envoyé toutes les 30 minutes par RF au format Oregon.
La température correspond à la mesure effectuée,
L'humidité correspond au pourcentage d'occupation de la cuve estimée.

### Composants utilisés :
- Capteur Ultrason SR04
- Ecran LCD type Nokia 5110
- Emetteur RF 433 Mhz
- Un arduino nano est suffisant pour exécuter ce sketch (7800 octets)


### Ressources utilisées :
Protocoles Oregon Scientific et Arduino : Encodage  par Olivier Lebrun
<http://www.connectingstuff.net/blog/encodage-protocoles-oregon-scientific-sur-arduino/>

Affichage 4 Chiffres et barre de progression 
<http://blog.3d-logic.com/2012/08/26/digital-clock-on-arduino-uno-with-nokia-lcd-display/>