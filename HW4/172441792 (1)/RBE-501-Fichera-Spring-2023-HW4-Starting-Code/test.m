L1 = 0.412;
L2 = 0.154;
L4= 0.263;

S_body = [0 0 1 -cross([0 0 1],[L2 0 -L4-L1]);
     -1 0 0 -cross([ -1 0 0],[-L2 0 -L4]);
      0 0 0  0 0 1;
      0 0 1 -cross([0 0 1],[0 0 -L4]);
      0 1 0 -cross([0 1 0],[0 0 -L4]);
      0 0 1 -cross([0 0 1],[0 0 -L4])]
