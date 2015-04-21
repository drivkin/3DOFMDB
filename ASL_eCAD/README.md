# ASL_eCAD

## Description

This repository contains all of the general-purpose electrical CAD projects produced & used by the Autonomous Systems Lab at UCSC under the direction of Professor Gabriel Elkaim. Descriptions of each project are in their respective folders. In the root folder you will find the Eagle 5.11 library used by all of these projects.

## CAD Standards

The following are accepted standards for the circuits that we use:
 * Standard sizes:
   * SMD resistors should be 0805
   * SMD capacitors should be 1206
   * SMD LEDs should be 1206. We stock amber, green, and red.
 * For connectors:
   * For power use the POW_CONNECT part. There are vertical and right-angle connectors. V_in is always pin 1.
   * For general-use headers, we have 2x7 female .1" spaced headers.
   * For single-row .1" headers use the staggered pins. With plated vias these allow for no-solder connections and also make soldering them in easier.
   * If screw terminals are needed, we stock the 5.08mm ones.
 * Trace widths:
   * For standard traces use 16mil.
   * For power traces use 30mil as a minimum for power-carrying traces. There are online trace width calculators available to determine appropriate sizes for high-current applications.
   * For traces that connect through .1" double-row headers use 10mil to pass DRC.
 * General:
   * Use a ground-plane wherever possible.
   * Place all SMD components on one side so that skilleting or using a reflow oven is possible.