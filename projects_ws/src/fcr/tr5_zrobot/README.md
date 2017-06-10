# tr5_zrobot
================

Entrada
-------
Velocidade de movimento do robo.

Saida
-----
Exploração do mapa realizada pelo robô, preenchendo uma grade de ocupação para cada nó do mapa topológico.

Algoritmo
---------
O implementação codificada não foi retirada de nenhuma referência, foi criada e desenvolvida pelo autor.
Entretando, os algoritmos implementados são bem conhecidos na academia. 

Início

	atualiza callbacks
    
    controi mapa topologico do cic
    inicializa grids para cada nó do mapa e inicializa cada célula com valor 0.5 (indefinido)

    obtem velocidade do robô do usuário
    define caminho a ser percorrido pelos nós para percorrer todo o mapa

    enquanto usuário não sair
		atualiza callbacks
		mostra nó atual do robo caso tenha mudado

		se nó atual é um nó conhecido
			atualiza a grade de ocupação do nó atual
			mostra a imagem do nó na tela (OpenCV)

    	se robô encontrou posição de destino
    		sai do loop
    	fim se

		executa algoritmo de atualização ponto a ponto
		executa algoritmo de checagem por obstaculos

		se a atualização ponto a ponto precisa atualizar a orientação do robo
			publica a velocidade obtida do algoritmo ponto a ponto
			torna falso contador de atualização da orientação dado pelos obstaculos
			inicializa contador de atualização da orientação temporal com valor máximo

		senão se a checagem de obstáculos encontrou algum obstáculo
			publica a velocidade obtida do algoritmo de evitar obstáculos
			habilita contagem do contador pós obstaculo
			inicializa contadores com valor máximo

		senão
			se algum contador se esgotou
				seta flag para reorientar robô
				para a contagem do contador pós obstaculo
				inicializa contadores com valor máximo
			fim se

			publica velocidade do usuário com angular em zero
		fim se
		espera até completar a frequencia desejada
	fim enquanto

	salva o estado de todas as grades de ocupação dos nós em arquivo

fim


Descrição dos arquivos
----------------------

include/
    tr5_zrobot/
            |--> obstacle_avoidance.h : header da classe que implementa o desvio de obstáculos.
            |--> point_kinematics.h   : header da classe que implementa a decisão de trajetória.
            |--> occupancy_grid.h     : header da classe que controi, mostra e salva as grades de ocupação de cada mapa.
            |--> topological_map.h    : header da classe que implementa o mapa topológico.
            |--> trabalho.h 		  : header da classe que implementa o fluxo de execução.
launch/
    |--> pioneer3at.gazebo.launch: arquivo que inicia as simulações do pioneer no gazevo e rviz.
resources/
    | --> report.pdf: relatório do trabalho 3.
    | --> trabalho3.avi: video com a simulação do programa.
src/
    |--> obstacle_avoidance.cpp : source com o corpo dos metodos da classe ObstacleAvoidance definido em obstacle_avoidance.h.
    |--> point_kinematics.cpp 	: source com o corpo dos metodos da classe PointKinematics definido em point_kinematics.h.
    |--> occupancy_grid.h       : source com o corpo dos metodos da classe OccupancyGrid definido em occupancy_grid.h.
    |--> topological_map.cpp 	: source com o corpo dos metodos da classe TopologicalMap definido em topological_map.h.
    |--> trabalho.cpp 			: source com o corpo dos metodos da classe Trabalho definido em trabalho.h.
    |--> node.cpp 				: No do ROS que comanda o pioneer com navegação e desvio de obstáculos.

CMakeLists.txt: Arquivo de configuração da build deste pacote.
package.xml: Arquivo de configuração de dependências deste pacote e informações de versão, autor e descrição.
README.md: Este arquivo.
trabalho.pro: opção para poder ver o arquivos fonte como um projeto no Qt.


