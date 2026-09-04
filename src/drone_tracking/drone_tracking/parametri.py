#!/usr/bin/env python3
"""Lettura dei parametri di taratura, con aggiornamento a caldo.

Le costanti che sono state ritarate piu volte durante lo sviluppo vivono come
parametri ROS 2 e non come letterali nel codice. La differenza pratica e
doppia: si cambiano senza ricompilare, e `ros2 param get` risponde con il
valore che era davvero attivo durante una prova, cosa che un numero scritto in
un file sorgente non puo fare per una prova gia conclusa.

    ros2 param list /controller_node
    ros2 param get  /controller_node kp_x
    ros2 param set  /controller_node kp_x 2.0

L'ultimo comando ha effetto immediato: `parametro()` registra l'associazione
fra nome del parametro e attributo del nodo, e installa una callback che
riassegna l'attributo quando il valore cambia. Senza quella callback il
parametro veniva letto solo nel costruttore e `ros2 param set` cambiava un
valore che nessuno rileggeva piu: e successo, e la prova che ne dipendeva ha
misurato la configurazione di partenza credendo di misurarne un'altra.

Attenzione: non tutti i parametri hanno senso a caldo. Il periodo di un timer
gia creato non cambia, e il seme di un generatore pseudocasuale a meta prova
non ha significato. Quelli che contano sono i guadagni, le soglie e le
velocita, che vengono riletti dai rispettivi attributi a ogni ciclo.

I parametri di ArduPilot (WP_SPD, ATC_ANGLE_MAX, SIM_*) sono un'altra cosa e
stanno altrove, in docker/sitl-defaults.parm: configurano l'autopilota, non i
nodi ROS, e non si leggono con `ros2 param`.
"""
from rcl_interfaces.msg import SetParametersResult
from rclpy.exceptions import ParameterAlreadyDeclaredException


def parametro(nodo, nome, default, attributo=None):
    """Dichiara il parametro, lo assegna al nodo come attributo e lo restituisce.

    Il tipo del default fissa il tipo del parametro: con un default 1.2 il
    parametro accetta solo numeri in virgola mobile, con 15 solo interi. Va
    quindi scritto 5.0 e non 5 dove il valore e una grandezza continua,
    altrimenti `ros2 param set ... 4.5` viene rifiutato a runtime.

    `attributo` serve nei pochi casi in cui il nome dell'attributo differisce da
    quello del parametro.
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
    """Callback di ROS: riassegna gli attributi dei parametri modificati.

    Viene chiamata *prima* che il valore sia accettato, quindi si assegna qui e
    si risponde che la modifica e valida. Un parametro non registrato viene
    accettato senza toccare nulla: puo essere `use_sim_time` o un parametro
    gestito direttamente dal nodo.
    """
    mappa = getattr(nodo, '_parametri_collegati', {})
    for p in parametri:
        attributo = mappa.get(p.name)
        if attributo is not None:
            setattr(nodo, attributo, p.value)
            nodo.get_logger().info(
                '{} = {}'.format(p.name, p.value))
    return SetParametersResult(successful=True)
