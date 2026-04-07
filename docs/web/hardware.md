# Hardware (fuente web)

Fuente principal: https://alejoblandon22.wixsite.com/robot-scara/blank-3
Fecha de integracion: 2026-04-06

## Arquitectura de hardware

El sitio reporta un sistema compuesto por actuadores, sensores, etapa de potencia y microcontrolador. La plataforma central es Raspberry Pi Pico (RP2040), con dos motores Pololu con encoder para articulaciones rotacionales y un servomotor MG90S para el eje prismativo de herramienta.

## Elementos funcionales

- Microcontrolador: Raspberry Pi Pico
- Actuadores: 2 motores Pololu 200 RPM + 1 servo MG90S
- Driver: L298N para control de motores DC
- Sensores: encoders y finales de carrera
- Interconexion: PCB propia y esquema de conexion

## Criterio de alimentacion

Se menciona operacion a 12V para la etapa de motores, buscando equilibrio entre desempeno, costo y facilidad de implementacion.

## Evidencia en la web

La pagina incluye bloques de PCB, esquema y conexion, junto con un diagrama grafico de conexiones de hardware.
