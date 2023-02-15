use example_msg::action::*;
use safe_drive::{self, action, context::Context, error::DynError, RecvResult};
use std::{thread, time::Duration};

fn main() -> Result<(), DynError> {
    let ctx = Context::new()?;

    let node_client = ctx.create_node("test_action_client_node", None, Default::default())?;

    let mut client: action::client::Client<MyAction> =
        action::client::Client::new(node_client, "test_action", None)?;

    thread::sleep(Duration::from_millis(100));

    // wait for action server
    // loop {
    // if client.is_server_available()? {
    // break;
    // }
    // }

    // send goal request
    let goal = MyAction_Goal { a: 100 };
    let mut goal_id = unique_identifier_msgs::msg::UUID::new().unwrap();

    goal_id.uuid = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16];

    let goal_request = MyAction_SendGoal_Request {
        // TODO: ergonomic apis
        // TODO: generate UUID w/ uuid crate. rclcpp's ClientBase::generate_goal_id
        // does not conform to the UUID v4 standard, strictly speaking.
        goal_id,
        goal,
    };
    client.send_goal_request(
        &goal_request,
        Box::new(|resp| {
            println!("Goal response received: {:?}", resp);
        }),
    )?;

    // receive goal response
    loop {
        match client.try_recv_goal_response() {
            // we wait for a single response here
            RecvResult::Ok(_) => break,
            RecvResult::RetryLater(_) => {
                println!("retrying...");
            }
            RecvResult::Err(e) => {
                // panic!("Error: {}", e);
                println!("Error: {}", e);
            }
        }
        std::thread::sleep(std::time::Duration::from_secs(1));
    }

    // get feedback

    // get result

    Ok(())
}
