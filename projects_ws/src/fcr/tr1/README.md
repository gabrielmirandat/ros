# tr1
================

Entrada
-------
Comandos de movimento teleoperado (geometry_msgs/Twist).

Saida
-----
Pioneer dando prioridade a evitar obstáculos mesmo com os comandos teleoperados.

Algoritmo
---------
O algoritmo implementado não foi retirado de nenhuma referência, foi criado e desenvolvido pelo autor.

Início
    inicia flag de ultima ação autônoma como falsa.

    enquanto verdade

    	inicia flag de obstrução como falsa.
        
        se leitura do laser adequada
        	para quarta parte inicial e quarta parte final das amostras do laser
        		armazena as amostras de doze em doze medidas
        		se alguma dessas amostras medir uma distância inferior à 0.8 metros, aciona flag de obstrução.
        	fim para

        	para a metade da frente do robo das amostras do laser
        		armazena as amostras de oito em oito medidas
        		se alguma dessas amostras medir uma distância inferior à um metro e meio, aciona flag de obstrução.
        	fim para

        	velocidade de escape é a maior velocidade linear em x ou angular em z de antes da obstrução.

        	se flag de obstrução ativa
        		nova velocidade linear recebe nulo.
        		nova velocidade angular recebe a velocidade de escape.
        		flag de última ação autônoma recebe verdadeiro.
        	senão
        	se a flag de última ação autônoma for verdade e a velocidade linear for zero 
        		nova velocidade linear recebe a velocidade de escape.
        		nova velocidade angular recebe nulo.
        		flag de última ação autônoma recebe falso.
        	fim se
		fim se

		atualiza velocidades de saída.
        envia velocidades de saída.
    fim enquanto
fim

Descrição dos arquivos
----------------------

include/
    tr1/
            |--> obstacle_avoidance.h: header da classe que implementa o controle do pioneer para desvio de obstáculos.
launch/
    |--> pioneer3at.gazebo.launch: arquivo que inicia as simulações do pioneer no gazevo e rviz.
    |--> teleop.launch: arquivo que inicia a teleoperação com o pioneer pelo teclado.
resources/
    | --> video: vídeo da simulação do trabalho.
    | --> report.pdf: relatório do trabalho 1.
src/
    |--> obstacle_avoidance.cpp: source com o corpo dos metodos da classe ObstacleAvoidance definido em obstacle_avoidance.h.
    |--> obstacle_avoidance_node.cpp: No do ROS que comanda o pioneer com movimento e desvio de obstáculos.
CMakeLists.txt: Arquivo de configuração da build deste pacote.
package.xml: Arquivo de configuração de dependências deste pacote e informações de versão, autor e descrição.
README.md: Este arquivo.
trabalho1.pro: opção para poder ver o arquivos fonte como um projeto no Qt.


