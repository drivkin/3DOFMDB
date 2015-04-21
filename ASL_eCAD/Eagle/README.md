This folder contains helper files for use with the Eagle eCAD tool.

Add the following directories to Eagle's configuration options to enable the use of these files.

== Folders ==
 /cam: Contains output file scripts.
   4pcb.cam: This CAM file outputs the necessary files to manufactoring via 4pcb.com
   cream.cam: Used to output the cream layers of boards. Used for skilleting boards.
   boardrouter.cam: This script provides the necessary output for working with the boardrouter.
   lasercuter.com: Outputs an outline of the board suitable for cutting in the laster cutter. This can be used for cutting acrylic masks that, when secured to the top and bottom of the board, allow one to run it through a router to trim any excess board.
 /lbr: Contains part libraries.
   ASL.lbr: The standard monolithic library used by the ASL lab. Many of these parts are actively stocked.
