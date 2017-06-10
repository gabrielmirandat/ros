# tr2
================

Entrada
-------
Velocidade de movimento do robo.
Destino especificando x,y.

Saida
-----
Navegação do robô para o ponto especificado evitando obstáculos.

Algoritmo
---------
O algoritmo implementado não foi retirado de nenhuma referência, foi criado e desenvolvido pelo autor.
Para o algoritmo de menor caminho de Dijkstra, uma implementação pronta foi obtida e modificada para usar no conjunto do código.

Início
    controi mapa topologico do cic
    cria grafo com pesos do cic (pesos obtidos com mediçoes no pdf do mapa)

    enquanto usuario não sair
		obtem velocidade do robô do usuário
	    obtem posição de destino informada pelo usuário
		obtem nó em que o robô se encontra atualmente no mapa topológico
	    obtém nó onde o destino se encontra
	    obtém o menor caminho de Dijkstra entre os nós inicial e final
		seta a posição intermediária de destino como a primeira posição do vetor do caminho

	    enquanto usuário não sair

	    	atualiza callbacks

	    	mostra nó atual do robo caso tenha mudado

	    	se robô encontrou posição de destino
	    		sai do loop mais interno
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

		pergunta se usuário quer mais uma iteração
	fim enquanto
fim


Descrição dos arquivos
----------------------

include/
    tr2/
            |--> graph.h 			  : header da classe que implementa a busca de menor caminho entre os nós.
            |--> obstacle_avoidance.h : header da classe que implementa o desvio de obstáculos.
            |--> point_kinematics.h   : header da classe que implementa a decisão de trajetória.
            |--> topological_map.h    : header da classe que implementa o mapa topológico.
            |--> trabalho.h 		  : header da classe que implementa o fluxo de execução.
launch/
    |--> pioneer3at.gazebo.launch: arquivo que inicia as simulações do pioneer no gazevo e rviz.
resources/
    | --> report.pdf: relatório do trabalho 2.
    | --> video.avi: video com a simulação do programa.
src/
    |--> graph.cpp 				: source com o corpo dos metodos da classe Graph definido em graph.h.
    |--> obstacle_avoidance.cpp : source com o corpo dos metodos da classe ObstacleAvoidance definido em obstacle_avoidance.h.
    |--> point_kinematics.cpp 	: source com o corpo dos metodos da classe PointKinematics definido em point_kinematics.h.
    |--> topological_map.cpp 	: source com o corpo dos metodos da classe TopologicalMap definido em topological_map.h.
    |--> trabalho.cpp 			: source com o corpo dos metodos da classe Trabalho definido em trabalho.h.
    |--> node.cpp 				: No do ROS que comanda o pioneer com navegação e desvio de obstáculos.

CMakeLists.txt: Arquivo de configuração da build deste pacote.
package.xml: Arquivo de configuração de dependências deste pacote e informações de versão, autor e descrição.
README.md: Este arquivo.
trabalho.pro: opção para poder ver o arquivos fonte como um projeto no Qt.