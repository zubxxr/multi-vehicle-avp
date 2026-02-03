This section summarizes system-level measurements collected during two-host and three-host DMV-AVP execution. It is important to note that observations seen here are similar to [DMAVA](https://zubxxr.github.io/distributed-multi-autonomous-vehicle-architecture/Scalability/SystemPerformanceAndNetworkCharacterization/). However, due to the addition of the U-YOLO and AVP-CF modules, system analysis will also be done here.

### Available System Memory for Two- and Three-Host Configurations

| Setup      | Host           | Available Memory |
|------------|----------------|------------------|
| Two-Host   | ROG Laptop     | 6.6 GiB         |
|            | Nitro PC       | 9.4 GiB          |
| Three-Host | ROG Laptop     | 6.0 GiB         |
|            | Nitro PC       | 9.4 GiB          |
|            | Victus Laptop  | 5.0 GiB          |

### CPU Utilization for Two-Host and Three-Host Configurations

| Configuration | Host           | Mean CPU (%) | Peak CPU (%) |
|---------------|----------------|--------------|--------------|
| Two-Host      | ROG Laptop     | 62           | < 95         |
|               | Nitro PC       | 48           | < 60         |
| Three-Host    | ROG Laptop     | 72           | < 95         |
|               | Nitro PC       | 54           | < 65         |
|               | Victus Laptop  | 46           | < 75         |


### RTT Measurements During Three-Host Active Operation Under Dedicated Local Access Point

| Configuration        | RTT (ms)        | Max RTT (ms) | Samples |
|----------------------|-----------------|--------------|---------|
| Two-Host             | 26.06 ± 14.53   | 100.74       | 2019    |
| Three-Host (ROG)     | 16.24 ± 15.02   | 93.89        | 1976    |
| Three-Host (Victus)  | 30.42 ± 17.53   | 138.11       | 1965    |


---

### Observations

See [DMAVA Observations](https://zubxxr.github.io/distributed-multi-autonomous-vehicle-architecture/Scalability/SystemPerformanceAndNetworkCharacterization/).