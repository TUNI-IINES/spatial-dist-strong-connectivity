# spatial-dist-strong-connectivity
 A Distributed Algorithm to Establish Strong Connectivity in Spatially Distributed Networks via Estimation of Strongly Connected Components

This repository stores the implementation of the proposed approaches in the following academic paper:

```text
@inproceedings{atman2024,
  title = {A Distributed Algorithm to Establish Strong Connectivity in Spatially Distributed Networks via Estimation of Strongly Connected Components},
  booktitle = {2024 European Control Conference (ECC)},
  author = {Atman, Made Widhi Surya and Gusrialdi, Azwirman},
  date = {2024-06},
  pages = {2493--2499},
  doi = {10.23919/ECC64448.2024.10590986},
  url = {https://ieeexplore.ieee.org/document/10590986},
  urldate = {2024-08-02},
  abstract = {This paper presents a distributed algorithm for ensuring the strong connectivity of spatially distributed networks where the communication network topology depends on both the position and communication range of the nodes. This is achieved by adding new links via adjusting the communication range and/or controlling the position of the nodes. The distributed algorithms rely on the estimation of strongly connected components of a dynamic network topology, accomplished through the utilization of the maximum consensus algorithm. The proposed strategies are scalable and converge in a finite number of steps without requiring information on the overall network topology. Finally, the proposed distributed algorithm is demonstrated through two case studies of ensuring strong connectivity in wireless networks with static and mobile nodes.},
  eventtitle = {2024 {{European Control Conference}} ({{ECC}})},
}
```

Running the code require the `nebosim` python library (TODO: the link will be added later).
