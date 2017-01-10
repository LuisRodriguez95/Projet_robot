#include "fonctions.h"
/* #DEFINE utilisés dans ce code : */
#define STATUS_NOK 2  // STATUS_OK=0
#define SEUIL_PERTE 2
int write_in_queue(RT_QUEUE *msgQueue, void * data, int size);

void envoyer(void * arg) {
    DMessage *msg;
    int err;

    while (1) {
        rt_printf("tenvoyer : Attente d'un message\n");
        if ((err = rt_queue_read(&queueMsgGUI, &msg, sizeof (DMessage), TM_INFINITE)) >= 0) {
            rt_printf("tenvoyer : envoi d'un message au moniteur\n");
            serveur->send(serveur, msg);
            msg->free(msg);
        } else {
            rt_printf("Error msg queue write: %s\n", strerror(-err));
        }
    }
}

void connecter(void * arg) {
    int status;
    DMessage *message;

    rt_printf("tconnect : Debut de l'exécution de tconnect\n");

    while (1) {
        rt_printf("tconnect : Attente du sémarphore semConnecterRobot\n");
        rt_sem_p(&semConnecterRobot, TM_INFINITE);
        rt_printf("tconnect : Ouverture de la communication avec le robot\n");
        status = robot->open_device(robot);

        rt_mutex_acquire(&mutexEtat, TM_INFINITE);
        etatCommRobot = status;
        rt_mutex_release(&mutexEtat);

        if (status == STATUS_OK) {
            status = robot->start_insecurely(robot);
            if (status == STATUS_OK){
                rt_printf("tconnect : Robot démarrer\n");
            }
        }

        message = d_new_message();
        message->put_state(message, status);

        rt_printf("tconnecter : Envoi message\n");
        message->print(message, 100);

        if (write_in_queue(&queueMsgGUI, message, sizeof (DMessage)) < 0) {
            message->free(message);
        }
    }
}

void communiquer(void *arg) {
    DMessage *msg = d_new_message();
    int var1 = 1;
    int num_msg = 0;

    rt_printf("tserver : Début de l'exécution de serveur\n");
    serveur->open(serveur, "8000");
    rt_printf("tserver : Connexion\n");

    rt_mutex_acquire(&mutexEtat, TM_INFINITE);
    etatCommMoniteur = 0;
    rt_mutex_release(&mutexEtat);

    while (var1 > 0) {
        rt_printf("tserver : Attente d'un message\n");
        var1 = serveur->receive(serveur, msg);
        num_msg++;
        if (var1 > 0) {
            switch (msg->get_type(msg)) {
                case MESSAGE_TYPE_ACTION:
                    rt_printf("tserver : Le message %d reçu est une action\n",
                            num_msg);
                    DAction *action = d_new_action();
                    action->from_message(action, msg);
                    switch (action->get_order(action)) {
                        case ACTION_CONNECT_ROBOT:
                            rt_printf("tserver : Action connecter robot\n");
                            rt_sem_v(&semConnecterRobot);
                            break;
                    }
                    break;
                case MESSAGE_TYPE_MOVEMENT:
                    rt_printf("tserver : Le message reçu %d est un mouvement\n",
                            num_msg);
                    rt_mutex_acquire(&mutexMove, TM_INFINITE);
                    move->from_message(move, msg);
                    move->print(move);
                    rt_mutex_release(&mutexMove);
                    break;
            }
        }
    }
}

void deplacer(void *arg) {
    int etatRobot; // Pour savoir l'etat de communication avec le robot
	int compteurPerte; // CompteurPerte
	int transmission; // Pour savoir s'il y'a transmission de set_motor ou pas
    int gauche;
    int droite;
    DMessage *message;

    rt_printf("tmove : Debut de l'éxecution de periodique à 1s\n");
    rt_task_set_periodic(NULL, TM_NOW, 1000000000);

    while (1) {
        /* Attente de l'activation périodique */
        rt_task_wait_period(NULL);
        //rt_printf("tmove : Activation périodique\n");

        rt_mutex_acquire(&mutexEtat, TM_INFINITE);
        etatRobot = etatCommRobot;
        rt_mutex_release(&mutexEtat);
	
		if (etatRobot != STATUS_OK) {
			rt_printf("tmove : Etat communication avec Robot DOWN \n");
		}
		else { 
			//Valeur de compteurPerte? (global donc prise de mutex)
			rt_mutex_acquire(&mutexCompteur, TM_INFINITE);
		    compteurPerte = compteurPerteRobot;
		    rt_mutex_release(&mutexCompteur);
		
			if (compteurPerte<SEUIL_PERTE) { //Envoie du message
		        rt_mutex_acquire(&mutexMove, TM_INFINITE);
		        switch (move->get_direction(move)) {
		            case DIRECTION_FORWARD:
		                gauche = MOTEUR_ARRIERE_LENT;
		                droite = MOTEUR_ARRIERE_LENT;
		                break;
		            case DIRECTION_LEFT:
		                gauche = MOTEUR_ARRIERE_LENT;
		                droite = MOTEUR_AVANT_LENT;
		                break;
		            case DIRECTION_RIGHT:
		                gauche = MOTEUR_AVANT_LENT;
		                droite = MOTEUR_ARRIERE_LENT;
		                break;
		            case DIRECTION_STOP:
		                gauche = MOTEUR_STOP;
		                droite = MOTEUR_STOP;
		                break;
		            case DIRECTION_STRAIGHT:
		                gauche = MOTEUR_AVANT_LENT;
		                droite = MOTEUR_AVANT_LENT;
		                break;
		        }
		        rt_mutex_release(&mutexMove);
				//Utilisation d'un mutex d'envoie de message
				rt_mutex_acquire(&mutexEnvoiRobot, TM_INFINITE);
		     	transmission = robot->set_motors(robot, gauche, droite);
		   		rt_mutex_release(&mutexEnvoiRobot);

		      
		
				if (transmission == STATUS_OK) { //remise du compteur à 0
					rt_printf("tmove : transmission OK = %d \n", transmission);
					rt_mutex_acquire(&mutexCompteur, TM_INFINITE);
		   			compteurPerteRobot = 0;
		    		rt_mutex_release(&mutexCompteur);
				}
				else { // Incrementation du compteur
					rt_mutex_acquire(&mutexCompteur, TM_INFINITE);
		   			compteurPerteRobot = compteurPerte +1;
		    		rt_mutex_release(&mutexCompteur);
					rt_printf("tmove : Incrementation de compteur = %d, car erreur transmission = %d \n", compteurPerteRobot,transmission);
				}
			}
			else { // Depassement de compteur : La connexion est DOWN donc etatCommRobot=STATUS_NOTOK
				rt_printf("tmove : Depassement de compteur\n");
				rt_mutex_acquire(&mutexEtat, TM_INFINITE);
        		etatCommRobot = STATUS_NOK; //valeur = 2 
        		rt_mutex_release(&mutexEtat);
				
				message = d_new_message();
                message->put_state(message, STATUS_NOK);

                rt_printf("tmove : Envoi message\n");
                if (write_in_queue(&queueMsgGUI, message, sizeof (DMessage)) < 0) {
                    message->free(message);
                }
			}
		} // FIN bigELSE
    } //fin While
}

void checkbatterie(void *arg) {
 
	int etatRobot; // Pour savoir l'etat de communication avec le robot
	int compteurPerte; // CompteurPerte
	int transmission; // Pour savoir s'il y'a transmission de set_motor ou pas
    DMessage *message;
	DBattery *batterie;
	batterie = d_new_battery();
	int vbatterie=10;
    rt_printf("tbatterie : Debut de l'éxecution de periodique à 1s\n");
    rt_task_set_periodic(NULL, TM_NOW, 1500000000);

    while (1) {
        /* Attente de l'activation périodique */
        rt_task_wait_period(NULL);
        //rt_printf("tmove : Activation périodique\n");

        rt_mutex_acquire(&mutexEtat, TM_INFINITE);
        etatRobot = etatCommRobot;
        rt_mutex_release(&mutexEtat);
	
		if (etatRobot != STATUS_OK) {
			rt_printf("tbatterie : Etat communication avec Robot DOWN \n");
		}
		else { 
			//Valeur de compteurPerte? (global donc prise de mutex)
			rt_mutex_acquire(&mutexCompteur, TM_INFINITE);
		    compteurPerte = compteurPerteRobot;
		    rt_mutex_release(&mutexCompteur);
		
			if (compteurPerte<SEUIL_PERTE) { //Envoie du message
		        //SECTION CRITIQUE
				rt_printf("tbatterie : transmission\n");
							
				rt_mutex_acquire(&mutexEnvoiRobot, TM_INFINITE);
		     	transmission=robot->get_vbat(robot,&vbatterie);
		   		rt_mutex_release(&mutexEnvoiRobot);

				
				batterie->set_level(batterie,vbatterie); // faudrait un mutex ? 

						
				if (transmission == STATUS_OK) { //remise du compteur à 0
					rt_printf("tbatterie : OK : niveau batterie : %d\n", vbatterie);
					rt_mutex_acquire(&mutexCompteur, TM_INFINITE);
		   			compteurPerteRobot = 0;
		    		rt_mutex_release(&mutexCompteur);
					
					/*
					message = d_new_message();
					message->put_state(message, STATUS_OK);
                	//message->put_battery_level(message, batterie);
               		if (write_in_queue(&queueMsgGUI, message, sizeof (DMessage)) < 0) {
                    	message->free(message);
                	} */
				}
				else { // Incrementation du compteur
					rt_mutex_acquire(&mutexCompteur, TM_INFINITE);
		   			compteurPerteRobot = compteurPerte +1;
		    		rt_mutex_release(&mutexCompteur);
					rt_printf("tmove : Incrementation de compteur = %d, car erreur transmission = %d \n", compteurPerteRobot,transmission);
				}
			}
			else { // Depassement de compteur : La connexion est DOWN donc etatCommRobot=STATUS_NOTOK
				rt_printf("tmove : Depassement de compteur\n");
				rt_mutex_acquire(&mutexEtat, TM_INFINITE);
        		etatCommRobot = STATUS_NOK; //valeur = 2 
        		rt_mutex_release(&mutexEtat);
				//ERREUR
				message = d_new_message();
                message->put_state(message, STATUS_NOK);

                rt_printf("tbatterie : Envoi message\n");
                if (write_in_queue(&queueMsgGUI, message, sizeof (DMessage)) < 0) {
                    message->free(message);
                }
			}
		} // FIN bigELSE
    } //fin While
}

int write_in_queue(RT_QUEUE *msgQueue, void * data, int size) {
    void *msg;
    int err;

    msg = rt_queue_alloc(msgQueue, size);
    memcpy(msg, &data, size);

    if ((err = rt_queue_send(msgQueue, msg, sizeof (DMessage), Q_NORMAL)) < 0) {
        rt_printf("Error msg queue send: %s\n", strerror(-err));
    }
    rt_queue_free(&queueMsgGUI, msg);

    return err;
}
