use std::time::Duration;
use rumqttc::{Event, Incoming, Publish};
use rumqttc::{MqttOptions, AsyncClient, QoS, EventLoop};
use serde::Deserialize;
use tokio::sync::Mutex;
use std::fs;
use tokio::select;
use tokio::time::{self};
use std::time::Instant;

use crate::rover::CommandState;

// Define a struct that mirrors your YAML structure
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
    client: Mutex<AsyncClient>,
    eventloop: Mutex<EventLoop>,
    state: Mutex<LinkState>,
}

impl ControlLink {
    pub fn new() -> ControlLink {
        let filename = "mqtt-server.yml";
        let contents = fs::read_to_string(filename)
            .expect("Could not read mqtt-server.yml file");
        let host_info: MqttHost = serde_yaml::from_str(&contents)
            .expect("Could not parse YAML");

        let mut mqttoptions = MqttOptions::new("rumqtt-async", host_info.host, host_info.port);
        mqttoptions.set_credentials(host_info.user, host_info.password);
        mqttoptions.set_keep_alive(Duration::from_secs(5));

        println!("RobotLink starting");
        let (client, eventloop) = AsyncClient::new(mqttoptions, 10);

        let state = LinkState { 
            last_heartbeat: Instant::now(),
            command_state: CommandState::default(),
        };

        ControlLink { client: Mutex::new(client), eventloop: Mutex::new(eventloop), state: Mutex::new(state) }
    }

    pub async fn handle_msg(&self, msg: &Publish) {
        println!("Received = {:?}", msg);
        match msg.topic.as_str() {
            "controller/heartbeat" => {
                // Got heartbeat from robot
                let mut state = self.state.lock().await;
                state.last_heartbeat = Instant::now();
                println!("Updated heartbeat time.");
            }
            "robot/command_state" => {
                // Got command / state from controller
                let mut state = self.state.lock().await;
                state.command_state = CommandState::default();  // TODO-DW : parse yaml message.
                println!("Got (and ignored) command state.");
            }
            _ => {
                // Ignoring unrecognized topics
            }
        }
    }

    pub async fn run(&self) {
        {
            let client = self.client.lock().await;

            let _ = client.subscribe("controller/heartbeat", QoS::AtMostOnce).await;  // TODO: Handle errors
            let _ = client.subscribe("robot/command_state", QoS::AtMostOnce).await; // TODO: Handle errors
        }

        // Task to send heartbeats
        let heartbeat_task = async move {
            let mut interval = time::interval(Duration::from_secs(1));
            loop {
                interval.tick().await;
                {
                    let client = self.client.lock().await;

                    // Send heartbeat
                    // TODO: Handle error
                    let _ = client.publish("robot/heartbeat", QoS::AtLeastOnce, false, "still alive").await;
                    println!("heartbeat.");
                }
            }
        };

        // Task to handle MQTT events
        let mqtt_handler = async move {
            // Listen for messages from the robot
            println!("RobotLink polling event loop");
            let mut eventloop = self.eventloop.lock().await;
            loop {
                let notification = eventloop.poll().await;
                match notification {
                    Ok(notification) => {
                        match notification {
                            Event::Incoming(Incoming::Publish(msg)) => {
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
                                println!("Ignoring notification {:?}", notification);
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
    }

}