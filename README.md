# WIP Sollumz version with RDR support

Fork of Sollumz_RDR (`19f8d5b05715a9f0754c96074f3b9fcb78f3d477 - August 2025`)<br>

In theory, you should be able to import/export any YNV for CodeX.

Polygon flags and edge flags are based on the research done by dexyfex and Disquse. It's still unknown what some of them actually do and even if naming is accurate.<br>

I also added an option in the UI panel to generate navmeshes from selected YBN collisions. The world is divided into fixed 150x150 cells, and only navmeshes that fit inside those cells are generated.

<ins>Known bugs:</ins>
 * **AI stopping randomly**: This happens due to navmesh borders, I made a better version with perfect adjacency using stitch polys/adjacent areas but even then the issue, likley due to polygon count... I've decided to leave this simpler version instead and eventually I'll get back working on it later.
   
 * **YBN buildings unsupported**: AI will currently try to run through buildings and walls, as if only terrain exists. Using navblockers can help mitigate this issue.
