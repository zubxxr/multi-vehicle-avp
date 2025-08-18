# Scaling to More Vehicles

The framework can be extended beyond two vehicles by adding more hosts and namespaces. However, during testing with the available machines, certain issues were encountered that impacted performance.  

Despite these constraints, the framework remained fully operational. Instructions are given below, as well as an example using a third host.

See the [Third Host Example](https://zubxxr.github.io/multi-vehicle-framework/Scalability/ScalingToMoreVehicles/#third-host-example) in the framework to add another vehicle.

## General Scaling Procedure for AVP

To extend the AVP system on top of the framework, replicate the following for each additional vehicle/host:

1. **Install the AVP Node** on the new host.  
2. **Assign a unique vehicle ID** for the new vehicle as an argument when launching the node.
3. **Add an extra namespace argument** corresponding to that vehicle ID on the host running the AVP managers.  


---

### Multi-Vehicle AVP Simulation (3 Hosts)

<iframe width="560" height="315" src="https://www.youtube.com/embed/wmk8ivT6jFE?list=PL4MADLjXmDi1Q5XXCuFTEntWz1c_T50jd" title="YouTube video" frameborder="0" allowfullscreen></iframe>

---

## Discussion

Similarly to the [discussion](https://zubxxr.github.io/multi-vehicle-framework/Scalability/ScalingToMoreVehicles/#discussion) about the three host simulation of the AV framework, an issue was observed with the **Victus Laptop**, where the third vehicle did not begin moving immediately. At 3:55 in the video above, when the first vehicle had already parked and the second was close to finishing its parking, the third vehicle finally started driving to the drop-off zone.

Further analysis of memory usage and host performance can be found in the [Benchmarks](../Benchmarks/index.md) section.

