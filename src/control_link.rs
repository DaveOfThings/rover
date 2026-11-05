use tokio::sync::mpsc::{Sender};
use std::time::Duration;
use rumqttc::{Event, Incoming, Publish};
use rumqttc::{MqttOptions, AsyncClient, QoS, EventLoop, ClientError};
use serde::Deserialize;
use tokio::sync::Mutex;
use std::fs;
use tokio::select;
use tokio::time::{self};
use std::time::Instant;

use crate::rover::CommandState;

const LINK_TIMEOUT: Duration = Duration::from_secs(2);

#[derive(Debug, Deserialize)]
struct MqttHost {
    host: String,
    port: u16,
    user: String,
    password: String,
}

pub struct LinkState {
    last_heartbeat: Instant,
    command_state: CommandState,
}

pub struct ControlLink {
    quit_tx: Sender<()>,
    client: Mutex<AsyncClient>,
    eventloop: Mutex<EventLoop>,
    state: Mutex<LinkState>,
}

impl ControlLink {
    pub fn new(quit_tx: Sender<()>) -> ControlLink {
        let filename = "mqtt-server.yml";
        let contents = fs::read_to_string(filename)
            .expect("Could not read mqtt-server.yml file");
        let host_info: MqttHost = serde_yml::from_str(&contents)
            .expect("Could not parse YAML");

        let mut mqttoptions = MqttOptions::new("rover2", host_info.host, host_info.port);
        mqttoptions.set_credentials(host_info.user, host_info.password);
        mqttoptions.set_keep_alive(Duration::from_secs(5));

        // println!("RobotLink starting");
        let (client, eventloop) = AsyncClient::new(mqttoptions, 10);

        let state = LinkState { 
            last_heartbeat: Instant::now(),
            command_state: CommandState::default(),
        };

        ControlLink { quit_tx, client: Mutex::new(client), eventloop: Mutex::new(eventloop), state: Mutex::new(state) }
    }

    pub async fn handle_msg(&self, msg: &Publish) {
        // println!("Received = {:?}", msg);
        match msg.topic.as_str() {
            "robot/quit" => {
                // We've been told to exit the app
                println!("Got signal to quit.");
                if let Err(_e) = self.quit_tx.send(()).await {
                    // Failed to send Quit message.  Log this but carry on.
                };
            }
            "controller/heartbeat" => {
                // Got heartbeat from robot
                let mut state = self.state.lock().await;
                state.last_heartbeat = Instant::now();
                // println!("Updated heartbeat time.");
            }
            "robot/command_state" => {
                // Got command / state from controller
                let mut state = self.state.lock().await;
                state.command_state = serde_json::from_slice::<CommandState>(&msg.payload).expect("Could not parse JSON");
                // println!("Got command state: {:?}", msg.payload);
            }
            _ => {
                // Ignoring unrecognized topics
                println!("Ignoring topic: {:?}", msg.topic.as_str());
            }

        }
    }

    pub async fn get_command_state(&self) -> CommandState {
        let mut link_state = self.state.lock().await;

        // If heartbeat from teleop station has stopped, command the robot to stop
        if link_state.last_heartbeat.elapsed() > LINK_TIMEOUT {
            // println!("Heartbeat Timed out");
            link_state.command_state = CommandState::Disabled
        }

        link_state.command_state
    }

    pub async fn run(&self) -> Result<(), ClientError> {
        {
            let client = self.client.lock().await;

            client.subscribe("controller/heartbeat", QoS::AtMostOnce).await?;
            client.subscribe("robot/command_state", QoS::AtMostOnce).await?;
            client.subscribe("robot/quit", QoS::AtMostOnce).await?;
        }

        // Task to send heartbeats
        let heartbeat_task = async move {
            let mut interval = time::interval(Duration::from_secs(1));
            loop {
                interval.tick().await;
                {
                    let client = self.client.lock().await;

                    // Send heartbeat
                    if let Err(_e) = client.publish("robot/heartbeat", QoS::AtLeastOnce, false, "still alive").await {
                        // Problem sending heartbeat, TODO: log error
                    }
                    // println!("heartbeat sent.");
                }
            }
        };

        // Task to handle MQTT events
        let mqtt_handler = async move {
            // Listen for messages from the robot
            // println!("RobotLink polling event loop");
            let mut eventloop = self.eventloop.lock().await;
            loop {
                let notification = eventloop.poll().await;
                match notification {
                    Ok(notification) => {
                        match notification {
                            Event::Incoming(Incoming::Publish(msg)) => {
                                // Received a message
                                self.handle_msg(&msg).await;
                            }
                            Event::Outgoing(_) => {
                                // println!("Don't need outgoing notification.");
                            }
                            Event::Incoming(Incoming::PubAck(_)) => {
                                // println!("Don't need to handle pub acks.");
                            }
                            Event::Incoming(Incoming::PingResp) => {
                                // println!("Don't need to handle ping responses.");
                            }
                            _ => {
                                // println!("Ignoring notification {:?}", notification);
                            }
                        }
                    }
                    Err(e) => {
                        println!("Error from eventloop.poll: {e:?}");
                    }
                }
            }
        };

        // Activate these two tasks when run() is awaited.
        // Neither should ever end.  If one does, run() terminates
        select! {
            _ = heartbeat_task => { println!("Stopped sending heartbeat."); },
            _ = mqtt_handler => { println!("MQTT handler quit."); }
        }
        
        println!("RobotLink ending");

        Ok(())
    }

}