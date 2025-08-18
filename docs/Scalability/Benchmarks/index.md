To evaluate the performance of running three vehicles across separate hosts, memory usage was recorded using `free -h` during execution. The results provide insight into system load and limitations when scaling beyond two vehicles.

### Raw Memory Snapshots (Three-Host Setup)

**Nitro PC (Host 1)**  

```bash
ovin@ovin-Nitro-N50-640:~$ free -h
                total        used        free      shared  buff/cache   available
Mem:             23Gi        17Gi       489Mi       126Mi       5.4Gi       5.4Gi
Swap:            31Gi          0B        31Gi
```

**ROG Laptop (Host 2)**  

```bash
zubair@zubair-ROG-Zephyrus-G15:~$ free -h
                total        used        free      shared  buff/cache   available
Mem:             22Gi        11Gi       5.8Gi       100Mi       5.7Gi        11Gi
Swap:            31Gi          0B        31Gi
```

**Victus Laptop (Host 3)**  

```bash
zubair@AVLab:~$ free -h
                total        used        free      shared  buff/cache   available
Mem:             15Gi        10Gi       276Mi       518Mi       4.8Gi       4.2Gi
Swap:            31Gi       0.0Ki        31Gi
```

---

### Condensed Comparison (Three-Host Setup)

| Host          | Total RAM | Used RAM | **Free RAM** | Available RAM | 
|---------------|-----------|----------|--------------|---------------|
| Nitro PC      | 23 Gi     | 17 Gi    | **489 Mi**   | 5.4 Gi        |
| ROG Laptop    | 22 Gi     | 11 Gi    | **5.8 Gi**   | 11 Gi         |
| Victus Laptop | 15 Gi     | 10 Gi    | **276 Mi**   | 4.2 Gi        |

---

### Observations

Observations are similar to the framework. See [Benchmarks](https://zubxxr.github.io/multi-vehicle-framework/Scalability/Benchmarks/).

---

### Baseline Comparison (Two-Host Setup)

**Nitro PC (Host 1)**  

```bash
ovin@ovin-Nitro-N50-640:~$ free -h
               total        used        free      shared  buff/cache   available
Mem:            23Gi        15Gi       426Mi       151Mi       7.1Gi      7.0Gi
Swap:           31Gi          0B        31Gi
```

**ROG Laptop (Host 2)**  

```bash
zubair@zubair-ROG-Zephyrus-G15:~$ free -h
               total        used        free      shared  buff/cache   available
Mem:            22Gi        12Gi       3.6Gi       212Mi       7.0Gi        10Gi
Swap:           31Gi          0B        31Gi
```

---

### Condensed Comparison (Two-Host Setup)

| Host          | Total RAM | Used RAM | **Free RAM** | Available RAM | 
|---------------|-----------|----------|--------------|---------------|
| Nitro PC      | 23 Gi     | 15 Gi    | **426 Mi**   | 7.0 Gi        |
| ROG Laptop    | 22 Gi     | 12 Gi    | **3.6 Gi**   | 10 Gi         |

---

## Summary Across Setups

To provide a high-level comparison, the following table consolidates **free memory** measurements across both the three-host and two-host experiments. This highlights how resource availability directly influenced system scalability.  

| Setup         | Host        | Free Memory After Launch |
|---------------|-------------|--------------------------|
| **Three-Host**| Nitro PC    | **489 MiB**  |
|               | ROG Laptop  | 5.8 GiB      |
|               | Victus      | **276 MiB**  |
| **Two-Host**  | Nitro PC    | **426 MiB**  |
|               | ROG Laptop  | 3.6 GiB      |  

> **Note:** Memory usage can vary between system reboots and runtime conditions. The three-host and two-host measurements were taken at different times, so small differences may reflect background processes or system state in addition to AVP load.