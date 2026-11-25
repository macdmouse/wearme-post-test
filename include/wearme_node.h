// wearme_node.h
// ===================================================================================================================
//  EL ÚNICO ARCHIVO QUE SE EDITA AL PROGRAMAR UNA TARJETA: el número de nodo de esa tarjeta.
// ===================================================================================================================
// Las claves LoRaWAN de cada nodo están en wearme_credentials.h (fuera de git). Con este número el firmware toma las
// de ese nodo y, en el PRIMER arranque de la tarjeta, las graba en la flash junto con el nodo. Después la tarjeta
// conserva su identidad aunque se le cargue un firmware compilado para otro nodo (lo avisa al arrancar), así que para
// actualizar el firmware de cualquier tarjeta no hace falta tocar este número.
// Para cambiarle el nodo a una tarjeta ya programada: por serie NODE:SET=<número> y reiniciar.
//
//   Nodo 3 -> NODO-003, Bluetooth "WearMe_12_N003"

#ifndef WEARME_NODE_H
#define WEARME_NODE_H

#define WEARME_NODE 5 // <-- número del nodo a programar (debe estar en wearme_credentials.h)

#define WEARME_HW_VERSION "12" // versión de la tarjeta Wearme: va en el nombre Bluetooth (WearMe_12_N003)

#endif