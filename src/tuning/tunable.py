import ntcore

class TunableDouble:
    def __init__(self, name: str, default_value: float, table_name: str = "SmartDashboard"):
        # 1. Get the table and topic
        inst = ntcore.NetworkTableInstance.getDefault()
        table = inst.getTable(table_name)
        self.topic = table.getDoubleTopic(name)
        
        # 2. Create publisher and subscriber
        # The publisher ensures the value exists in Glass immediately
        self.pub = self.topic.publish()
        self.pub.set(default_value)
        
        # The subscriber listens for changes from Glass
        self.sub = self.topic.subscribe(default_value)

    def get(self) -> float:
        """Get the current value from NetworkTables (updated by Glass)."""
        return self.sub.get()

    def set(self, value: float):
        """Manually update the value from the robot side."""
        self.pub.set(value)