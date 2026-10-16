# esphome-poort-t4

ESPHome-component voor een dubbele poort met twee Nice SNA20-sturingen via Bus T4.

- `components/bus_t4`: aangepaste versie van [makstech/esphome-BusT4](https://github.com/makstech/esphome-BusT4) (v1)
  met meerdere bussen tegelijk, busbewaking en UART-aanduiding in de log.
- `components/bus_t4/poort_logic.h`: logica voor de dubbele poort (volgorde, vergrendeling,
  maglocks, reservemodus via P.P.).

Gebruik in ESPHome:

```yaml
external_components:
  - source: github://stevefury/esphome-poort-t4@main
    components: [bus_t4]
    refresh: 0s
```
