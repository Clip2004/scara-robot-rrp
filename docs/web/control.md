# Control (fuente web)

Fuente principal: https://alejoblandon22.wixsite.com/robot-scara/about-3
Fecha de integracion: 2026-04-06

## Operacion del sistema

La seccion enfatiza control de dos variables ligadas: posicion y velocidad angular. Se plantea una estrategia de ajuste iterativo para alcanzar un punto de operacion estable, con restricciones de seguridad y calidad de trayectoria.

## Posicion

La posicion se considera variable critica e inestable, por lo que se requiere control preciso y respeto de limites fisicos de recorrido definidos por finales de carrera y geometria mecanica.

## Velocidad angular

Se prioriza evitar aceleraciones abruptas y cambios bruscos que puedan afectar calidad de soldadura, rebotes mecanicos o seguridad de operacion.

## Criterios de diseno en la pagina

- Suavidad de movimiento
- Seguridad del operador
- Cumplimiento de limites mecanicos
- Ajuste independiente por variable para converger a operacion robusta
