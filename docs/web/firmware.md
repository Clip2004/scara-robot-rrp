# Firmware (fuente web)

Fuente principal: https://alejoblandon22.wixsite.com/robot-scara/projects-6
Fecha de integracion: 2026-04-06

## Plataforma y alcance

El firmware fue implementado sobre Raspberry Pi Pico usando MicroPython para pruebas de laboratorio, validacion de rutinas y ejecucion de trayectorias.

## Funciones operativas reportadas

- Rutina de referencia (home)
- Movimiento a punto dentro del workspace
- Trayectoria lineal
- Trayectoria semicircular
- Uso de finales de carrera para seguridad de limites

## Interfaz de comandos

La pagina describe tres comandos principales:

- C: trayectoria semicircular
- L: trayectoria lineal
- P: ir a punto del workspace

## Limitaciones observadas

El contenido de resultados del sitio indica que MicroPython, al ser interpretado y sin RTOS, presenta limitaciones para tareas con temporizacion estricta y multitarea de alta exigencia.
