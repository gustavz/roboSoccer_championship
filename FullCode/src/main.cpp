//============================================================================
// Name        : soccer_client_1.cpp
// Author      :
// Version     :
// Copyright   : Your copyright notice
// Description : Client for RTDB which controls team 1, Ansi-style
//============================================================================

#include <time.h>
#include <iostream>
#include <thread>
#include "kogmo_rtdb.hxx"
#include "robo_control.h"

#include "lib/GoalKeeper.h"
#include "lib/FieldPlayer.h"
#include "lib/GameControl.h"
#include "lib/Debug.h"
#include "lib/Ball.h"

//#include <boost/shared_ptr.hpp>


using namespace std;

int main(int argc, char *argv[]) {
	//--------------------------------- Init --------------------------------------------------

	/** Use client number according to your lab_roso_stud account number!
	 *
	 *	This is necessary in order to assure that there are unique
	 *	connections to the RTDB.
	 *
	 */
        const int client_nr = 14;
      			

	try {

		/** Establish connection to the RTDB.
		 *
		 *  The connection to the RTDB is necessary in order to get access
		 *  to the control and the status of the robots which are both stored
		 *  in the RTDB.
		 *
		 *  In the RTDB there are also informations about the ball and the
		 *  other robot positions.
		 *
		 */
		cout << endl << "Connecting to RTDB..." << endl;
		/** Create the client name with the unique client number*/
                string client_name = "pololu_client_";
                client_name.push_back((char) (client_nr + '0'));
                RTDBConn DBC(client_name.data(), 0.1, "");

                Ball ball(DBC);
                Referee* ref(new Referee(DBC));
                ref->Init();

                Physics* physics(new Physics(&ball));


                // choose team colour
                int idOffset = 0;

                string teamColor = "b";
                std::cout << "Blue or Red? (b / r)" << std::endl;
				std::cin >> teamColor;

                if(teamColor == "b"){
                    idOffset = 0;
                }else{
                    idOffset = 3;
                }

                GoalKeeper* Goaly(new GoalKeeper(DBC,0+idOffset, physics));
                cout << "Our Robot: " << 0+idOffset << " done" << endl;
                FieldPlayer* Player1(new FieldPlayer(DBC,1+idOffset, physics));
                cout << "Our Robot: " << 1+idOffset << " done" << endl;
                FieldPlayer* Player2(new FieldPlayer(DBC,2+idOffset, physics));
                cout << "Our Robot: " << 2+idOffset << " done" << endl;

                Enemy* Enemy1(new Enemy(DBC,3-idOffset, physics));
                cout << "Robot: " << 3-idOffset << " done" << endl;
                Enemy* Enemy2(new Enemy(DBC,4-idOffset, physics));
                cout << "Robot: " << 4-idOffset << " done" << endl;
                Enemy* Enemy3(new Enemy(DBC,5-idOffset, physics));
                cout << "Robot: " << 5-idOffset << " done" << endl;

                physics->addEnemy(Enemy1);
                physics->addEnemy(Enemy2);
                physics->addEnemy(Enemy3);

                physics->addAgent(Goaly);
                physics->addAgent(Player1);
                physics->addAgent(Player2);

                GameControl* gameControl(new GameControl(ref, physics, (teamColor == "b") ? (BLUE_TEAM) : (RED_TEAM)));
                Debug* debug(new Debug(gameControl));

                // choose strategy mode
                STRATEGIES::Strategies permanentStrategy;
                bool autoStrategy = true;

                string gameStrategy;
                std::cout << "Game strategy: offensive, defensive or auto? (o / d / a)" << std::endl;
				std::cin >> gameStrategy;
                if(gameStrategy == "o"){
                        permanentStrategy = STRATEGIES::OFFENSIVE;
                        autoStrategy = false;
                }else if(gameStrategy == "d"){
                        permanentStrategy = STRATEGIES::DEFENSIVE;
                        autoStrategy = false;
                }

                if (!autoStrategy)
                {
                        gameControl->setPermanentStrategy(permanentStrategy);
                }

                physics->initializePhysics();

                srand(time(NULL));


		//-------------------------------------- Ende Init ---------------------------------

                gameControl->setGoalKeeper(Goaly);
                gameControl->setFieldPlayer1(Player1);
                gameControl->setFieldPlayer2(Player2);

                std::cout << "Init Done" << std::endl;


                thread t1(&Physics::run, physics);
                t1.detach();
                std::cout << "Physics started" << std::endl;

                thread t2(&GoalKeeper::run, Goaly);
                t2.detach();
                std::cout << "GoalKeeper started" << std::endl;

                thread t3(&Debug::run, debug);
                t3.detach();
                std::cout << "Debug started" << std::endl;

                thread t4(&FieldPlayer::run, Player1);
                t4.detach();
                std::cout << "FieldPlayer1 started" << std::endl;

                thread t5(&FieldPlayer::run, Player2);
                t5.detach();
                std::cout << "FieldPlayer2 started" << std::endl;

                gameControl->run();



	} catch (DBError err) {
		cout << "Client died on Error: " << err.what() << endl;
        } catch (...) {
            cout << "Unknown Error" << endl;
        }

	cout << "ende" << endl;


	return 0;
}

