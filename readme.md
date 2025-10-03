# R-D_Software_Magalhaes_V2
## How to add this repository to STM32CubeIDE
1. Double click the .project file and press YES in prompt
2. For every project in the repository (STM32...), double click its .project file and press YES in prompt
3. Verify if the projects have been imported into STM32CubeIDE and try compiling them

## How to add STM to repository
#### Creating .ioc file and project
1. create .ioc file
2. create config.h file in project file
3. configure .ioc file w/ help of other config.h files 
4. configure config.h file with **SAME** variable names in other config.h files and use pins and connections configured in .ioc
5. in .ioc file configure clock tree
6. in .ioc file project manager -> project -> Toolchain / IDE -> STM32CubeIDE
7. Generate Code (.ioc)

#### Adding external libraries (Codigo + STM32 Modular Drivers)
8. create config.h file in Core/Inc and fill it!
9. Adicionar um link do STM32 Modular Drivers na root do projeto
    - Right Click Project and select New -> Folder
    - Set _Folder name_: "STM32 Modular Drivers"
    - Select Advanced -> Link to alternate location (Linked Folder)
    - **In case of FreeRTOS**, write to textbox _"WORKSPACE_LOC\Eletro_Software_STM32_Modular_Drivers\FreeRTOS"_
    - Deve ter sido criado um softlink ao repositório. Verificar se as folders __Src__ e __Inc__ estão na pasta STM32 Modular Drivers
11. Adicionar um link do Codigo na root do projeto
    - Select New -> Folder
    - Set _Folder name_: "Codigo"
    - Select Advanced -> Link to alternate location (Linked Folder)
    - **In case of FreeRTOS**, write to textbox _"WORKSPACE_LOC\R-D_Software_Magalhaes_V2\Codigo"_
    - Deve ter sido criado um softlink ao repositório. Verificar se a folder tem ficheiros
10. Adicionar os ficheiros da pasta Src ao compilador
    - Go to Project -> Properties  -> C/C++ General -> Paths and Symbols -> Source Location
    - Select Add Folder...
    - Expand folder __STM32 Modular Drivers__ and select the __Src__ folder
    - Select Add Folder...
    - Select __Codigo__
11. Adicionar os ficheiros da pasta Inc ao compilador
    - Go to Project -> Properties  -> C/C++ General -> Paths and Symbols -> Includes
    - Select Add... -> Workspace...
    - Expand folder of your project, expand folder __STM32 Modular Drivers__ and select the __Inc__ folder
    - Select Add... -> Workspace...
    - Expand folder of your project, select folder __Codigo__
12. Remove Core/Src/syscalls.c from compiler
    - Right click file and select Properties
    - Go to C/C++ General -> Paths and Symbols and check Exclude resource from build
13. Do the same thing to Core/Inc/FreeRTOSConfig.h
    - Right click file and select Properties
    - Go to C/C++ General -> Paths and Symbols and check Exclude 
14. Do the same thing to FATFS/Target/user_diskio.c
    - Right click file and select Properties
    - Go to C/C++ General -> Paths and Symbols and check Exclude resource from build
15. Deves estar pronto para incluir os ficheiros no teu projeto e compilar!
    - Add to main.c file _#include "retarget.h"_
    - Add to main.c file _#include "create_threads.h"_
    - Add to main.c _RetargetInit(&huart);_ and _create_threads();_
    - Finalmente, compila o projeto. Se seguiste os passos corretamente, deves conseguir compilar sem erros
