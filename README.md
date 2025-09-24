# WIP Sollumz version with RDR support

Fork of Sollumz_RDR (`19f8d5b05715a9f0754c96074f3b9fcb78f3d477 - August 2025`)<br>
Someone asked me to add YNV support for their Mexico project so I did this.

Theorically you should be able to import/export any YNV for CodeX.<br>
Polygon flags and edge flags are based on the research done by dexyfex/Disquse, it's unknown what most does and if naming is even correct.<br>

I added an option in the UI panel to generate navmeshes from selected YBN collisions. This is partitioning the world into fixed 150x150 cells and only generating navmeshes that fit inside them.

<ins>Known bugs:</ins>
 * AI stopping randomly: This is due to navmesh borders, I made a better version with perfect adjacency with stitch polys/adjacent areas but even with that issues still occur, this is I guess due to polygon count... So I decided to leave this version instead and eventually I'll get back working on it later.
 * YBN buildings are unsupported: That means the AI will try to run through walls and such, like if there was only terrain (navblockers can be used to fix that issue).