extends Node3D

func _ready():
    print("=== AdvancedNavigation Test Lab ===")

    if ClassDB.class_exists("AdvancedAINavigator3D"):
        print("SUCCESS: AdvancedAINavigator3D is registered!")

        var navigator = AdvancedAINavigator3D.new()
        add_child(navigator)

        print("SUCCESS: Navigator instance created!")
        print("Steering mode: ", navigator.steering_mode)
        print("Maximum speed: ", navigator.max_speed)

    else:
        push_error("FAILED: AdvancedAINavigator3D is not registered!")
