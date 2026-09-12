#!/usr/bin/env python3
"""Lettura dei parametri di taratura, con aggiornamento a caldo.

Le costanti ritarate piu volte vivono come parametri ROS 2 e non come letterali:
si cambiano senza ricompilare, e `ros2 param get` risponde con il valore che era
davvero attivo durante una prova.

    ros2 param set /controller_node kp_x 2.0

Il comando ha effetto immediato perche `parametro()` registra l'associazione fra
nome del parametro e attributo del nodo e installa una callback che riassegna
l'attributo. Senza, il valore verrebbe letto solo nel costruttore.

Non tutto ha senso a caldo: il periodo di un timer gia creato non cambia, e un
seme pseudocasuale a meta prova non significa nulla. Valgono guadagni, soglie e
velocita, riletti dai rispettivi attributi a ogni ciclo.

I parametri di ArduPilot (WP_SPD, ATC_ANGLE_MAX, SIM_*) sono un'altra cosa e
stanno in docker/sitl-defaults.parm: configurano l'autopilota, non i nodi ROS.
"""
from rcl_interfaces.msg import SetParametersResult
from rclpy.exceptions import ParameterAlreadyDeclaredException


def parametro(nodo, nome, default, attributo=None):
    """Dichiara il parametro, lo assegna al nodo come attributo e lo restituisce.

    Il tipo del default fissa il tipo del parametro: va scritto 5.0 e non 5 dove
    il valore e una grandezza continua, altrimenti `ros2 param set ... 4.5`
    viene rifiutato a runtime.
    """
    try:
        nodo.declare_parameter(nome, default)
    except ParameterAlreadyDeclaredException:
        pass

    _registra(nodo, nome, attributo or nome)
    valore = nodo.get_parameter(nome).value
    setattr(nodo, attributo or nome, valore)
    return valore


def _registra(nodo, nome, attributo):
    """Tiene la mappa nome->attributo e installa la callback una volta sola."""
    mappa = getattr(nodo, '_parametri_collegati', None)
    if mappa is None:
        mappa = {}
        nodo._parametri_collegati = mappa
        nodo.add_on_set_parameters_callback(
            lambda parametri: _applica(nodo, parametri))
    mappa[nome] = attributo


def _applica(nodo, parametri):
    """Riassegna gli attributi dei parametri modificati.

    ROS chiama questa callback *prima* di accettare il valore, quindi si assegna
    qui e si risponde che la modifica e valida. Un parametro non registrato
    passa senza toccare nulla: puo essere `use_sim_time`.
    """
    mappa = getattr(nodo, '_parametri_collegati', {})
    for p in parametri:
        attributo = mappa.get(p.name)
        if attributo is not None:
            setattr(nodo, attributo, p.value)
            nodo.get_logger().info('{} = {}'.format(p.name, p.value))
    return SetParametersResult(successful=True)
