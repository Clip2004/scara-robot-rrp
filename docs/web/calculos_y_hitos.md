# Calculos, derivaciones y hitos del desarrollo (integracion)

Fuentes:
- https://alejoblandon22.wixsite.com/robot-scara
- https://alejoblandon22.wixsite.com/robot-scara/about-3
- https://alejoblandon22.wixsite.com/robot-scara/copia-de-arquitectura-firmware-hardware
Fecha de integracion: 2026-04-06

## Donde estan los calculos en el repositorio

La web publica una vista general del proyecto, mientras que el detalle matematico se conserva en documentos tecnicos del repositorio:

- docs/dh_parameters.md
- docs/kinematics.md
- docs/control_design.md

## Trazabilidad tecnica

La narrativa web y los documentos locales son consistentes en los siguientes puntos:

- Modelo SCARA RRP de 3 GDL
- Integracion de cinemativa directa/inversa para trayectorias
- Control de posicion y velocidad angular
- Validacion experimental con rutinas home, lineal y semicircular

## Hitos del desarrollo (resumen)

1. Definicion del problema de automatizacion para soldadura.
2. Diseno mecanico preliminar y seleccion de arquitectura SCARA.
3. Integracion de hardware (motores, encoders, Pico, driver, limites).
4. Implementacion de firmware y comandos C/L/P.
5. Ajuste de control para posicion y velocidad.
6. Validacion final con trayectorias y reporte de conclusiones.

## Nota

Este archivo funciona como puente entre el contenido descriptivo del sitio web y los documentos matematicos detallados ya presentes en el repositorio.
