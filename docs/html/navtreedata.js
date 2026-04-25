/*
 @licstart  The following is the entire license notice for the JavaScript code in this file.

 The MIT License (MIT)

 Copyright (C) 1997-2020 by Dimitri van Heesch

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 and associated documentation files (the "Software"), to deal in the Software without restriction,
 including without limitation the rights to use, copy, modify, merge, publish, distribute,
 sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all copies or
 substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
 BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

 @licend  The above is the entire license notice for the JavaScript code in this file
*/
var NAVTREE =
[
  [ "Magalhães Flight Computer", "index.html", [
    [ "Start here", "index.html#autotoc_md98", null ],
    [ "Reference", "index.html#autotoc_md99", null ],
    [ "At a glance", "index.html#autotoc_md101", null ],
    [ "Hardware", "index.html#autotoc_md103", null ],
    [ "Authors and licence", "index.html#autotoc_md105", null ],
    [ "Getting Started", "d1/d06/getting_started.html", [
      [ "Prerequisites", "d1/d06/getting_started.html#autotoc_md84", null ],
      [ "Repository layout", "d1/d06/getting_started.html#autotoc_md85", null ],
      [ "Building the firmware", "d1/d06/getting_started.html#autotoc_md86", null ],
      [ "Talking to the board", "d1/d06/getting_started.html#autotoc_md87", null ],
      [ "Building the documentation", "d1/d06/getting_started.html#autotoc_md88", null ],
      [ "What to read next", "d1/d06/getting_started.html#autotoc_md89", null ]
    ] ],
    [ "Architecture Tour", "d5/d0f/architecture_tour.html", [
      [ "The big picture", "d5/d0f/architecture_tour.html#autotoc_md61", null ],
      [ "Thread cheat sheet", "d5/d0f/architecture_tour.html#autotoc_md62", null ],
      [ "Why these boundaries?", "d5/d0f/architecture_tour.html#autotoc_md63", [
        [ "Why DMA-driven, not polled?", "d5/d0f/architecture_tour.html#autotoc_md64", null ],
        [ "Why a separate DataHandler?", "d5/d0f/architecture_tour.html#autotoc_md65", null ],
        [ "Why TDMA on the radio?", "d5/d0f/architecture_tour.html#autotoc_md66", null ],
        [ "Why SPI on H743 but UART on F446ZE?", "d5/d0f/architecture_tour.html#autotoc_md67", null ],
        [ "Why no global state?", "d5/d0f/architecture_tour.html#autotoc_md68", null ]
      ] ],
      [ "Cortex-M7 gotcha (Buzz V4 only)", "d5/d0f/architecture_tour.html#autotoc_md69", null ],
      [ "Where things actually live", "d5/d0f/architecture_tour.html#autotoc_md70", null ],
      [ "Read next", "d5/d0f/architecture_tour.html#autotoc_md71", null ]
    ] ],
    [ "Flight Lifecycle", "dd/d01/flight_lifecycle.html", [
      [ "State diagram", "dd/d01/flight_lifecycle.html#autotoc_md72", null ],
      [ "State by state", "dd/d01/flight_lifecycle.html#autotoc_md73", [
        [ "BOOT", "dd/d01/flight_lifecycle.html#autotoc_md74", null ],
        [ "IDLE", "dd/d01/flight_lifecycle.html#autotoc_md75", null ],
        [ "CONFIGED", "dd/d01/flight_lifecycle.html#autotoc_md76", null ],
        [ "ARMED", "dd/d01/flight_lifecycle.html#autotoc_md77", null ],
        [ "TEST_STAND", "dd/d01/flight_lifecycle.html#autotoc_md78", null ],
        [ "FLIGHT", "dd/d01/flight_lifecycle.html#autotoc_md79", null ],
        [ "ABORT", "dd/d01/flight_lifecycle.html#autotoc_md80", null ],
        [ "SAFE", "dd/d01/flight_lifecycle.html#autotoc_md81", null ]
      ] ],
      [ "What \"an event\" means", "dd/d01/flight_lifecycle.html#autotoc_md82", null ],
      [ "When you're modifying the FSM", "dd/d01/flight_lifecycle.html#autotoc_md83", null ]
    ] ],
    [ "Porting Guide", "d0/d07/porting_guide.html", [
      [ "The abstraction contract", "d0/d07/porting_guide.html#autotoc_md90", [
        [ "Current macro contract", "d0/d07/porting_guide.html#autotoc_md91", null ]
      ] ],
      [ "Adding a new MCU target", "d0/d07/porting_guide.html#autotoc_md92", null ],
      [ "Swapping a sensor", "d0/d07/porting_guide.html#autotoc_md93", null ],
      [ "Changing peripheral mapping on an existing board", "d0/d07/porting_guide.html#autotoc_md94", null ],
      [ "Documentation auto-regeneration", "d0/d07/porting_guide.html#autotoc_md95", null ],
      [ "Where the doc structure itself is defined", "d0/d07/porting_guide.html#autotoc_md96", null ]
    ] ],
    [ "Todo List", "dd/d00/todo.html", null ],
    [ "Topics", "topics.html", "topics" ],
    [ "Namespaces", "namespaces.html", [
      [ "Namespace List", "namespaces.html", "namespaces_dup" ],
      [ "Namespace Members", "namespacemembers.html", [
        [ "All", "namespacemembers.html", null ],
        [ "Functions", "namespacemembers_func.html", null ],
        [ "Variables", "namespacemembers_vars.html", null ]
      ] ]
    ] ],
    [ "Data Structures", "annotated.html", [
      [ "Data Structures", "annotated.html", "annotated_dup" ],
      [ "Data Structure Index", "classes.html", null ],
      [ "Class Hierarchy", "hierarchy.html", "hierarchy" ],
      [ "Data Fields", "functions.html", [
        [ "All", "functions.html", "functions_dup" ],
        [ "Functions", "functions_func.html", null ],
        [ "Variables", "functions_vars.html", "functions_vars" ]
      ] ]
    ] ],
    [ "Files", "files.html", [
      [ "File List", "files.html", "files_dup" ],
      [ "Globals", "globals.html", [
        [ "All", "globals.html", "globals_dup" ],
        [ "Functions", "globals_func.html", "globals_func" ],
        [ "Variables", "globals_vars.html", "globals_vars" ],
        [ "Typedefs", "globals_type.html", null ],
        [ "Enumerations", "globals_enum.html", null ],
        [ "Enumerator", "globals_eval.html", "globals_eval" ],
        [ "Macros", "globals_defs.html", "globals_defs" ]
      ] ]
    ] ]
  ] ]
];

var NAVTREEINDEX =
[
"annotated.html",
"d1/d0d/lora__sx1276_8c.html#ac51cadc7c87c3b4ae21faebe2ccc8e27ae6e79fc4e02584bc3d353ac9c9c6e864",
"d2/d04/group___a_s_m330_l_h_h_x.html#ga3378cba44ba177f56262c6c0b799d492",
"d2/d04/group___a_s_m330_l_h_h_x.html#gab26ee77773af863d89ec068acfe1bb88",
"d2/d04/group___a_s_m330_l_h_h_x.html#gga7dc70b6766ad581b9658c7ff3f35d703a4fa86c0948bb08dadd1730a00a21171f",
"d3/d00/fatfs__sd_8c.html#a4547fab04380ea98a80691b8bdc8fc0e",
"d4/d0c/_s_t_m32_h743_z_i_t6___magalhaes_2_core_2_inc_2config_8h.html#a35c0a44611f4a31b6424c1cc3186513c",
"d5/d08/structasm330lhhx__wake__up__src__t.html#a52a6809d46b832dd1057f7f27c7197d4",
"d6/d0a/flash__data__handler_8h.html#a1520311fbbe175f79d33d9849971d580",
"d7/d0d/group___a_s_m330_l_h_h_x__free__fall.html#ga986c443c0af8aebbfee819869b15f303",
"d9/d0c/structasm330lhhx__mlc__status__mainpage__t.html",
"db/d02/structasm330lhhx__sh__cfg__read__t.html#abd40dec5c361031c89dacbdfad53083e",
"db/d0e/group___f_s_m_functions.html#ga351aabdf620e21f15f86191d9c1181f8",
"dd/d01/group___f_s_m.html#gga7dd5f9906807cad8b16104a7501a95cfa3c9a5c5ad3a321d90bb1395127b4f4b7",
"de/d03/_free_r_t_o_s_config_8h.html#ae8f3fd645e6e78dfeb8a6e874af6195a",
"de/d0b/unionasm330lhhx__reg__t.html#a70fa0cdb6ebd20d6881c19260852f580",
"df/d04/lora__sx1276_8h.html#ae783b6f8772e6fa497e29520b0a6606ba773e22108e9bbb59e0ad790484e44e35",
"dir_aa389fa14933c713f0e09709e6a0a834.html"
];

var SYNCONMSG = 'click to disable panel synchronization';
var SYNCOFFMSG = 'click to enable panel synchronization';
var LISTOFALLMEMBERS = 'List of all members';