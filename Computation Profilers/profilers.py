#!/usr/bin/env python3

import functools
import psutil
import rospy
from std_msgs.msg import Float32
from rosgraph_msgs.msg import Clock
import time  # Using time to measure computation time
import resource

def ns_join(*names):
    return functools.reduce(rospy.names.ns_join, names, "")

initial_RAM = initial_CPU = 0

class PerformanceMonitor:
    def __init__(self):
        rospy.init_node('performance_monitor')

        self.start_time = None
        self.comp_times = []
        self.cpu_percents = []
        self.ram_usages = []
        self.monitoring = False

        self.cpu_publish = rospy.Publisher("~cpu_load", Float32, queue_size=20)
        self.mem_publish = rospy.Publisher("~ram_usage", Float32, queue_size=20)
        self.comp_time_publish = rospy.Publisher("~computation_time", Float32, queue_size=20)

        rospy.on_shutdown(self.on_shutdown)
        self.wait_for_clock_topic()

    def wait_for_clock_topic(self):
        global initial_CPU, initial_RAM
        rospy.loginfo("Waiting for /clock topic to become available...")
        while not rospy.is_shutdown():
            topics = rospy.get_published_topics()
            topic_names = [t[0] for t in topics]
            if "/clock" in topic_names:
                rospy.loginfo("/clock topic detected. Starting performance monitoring.")
                initial_CPU = psutil.cpu_percent(interval=None)
                initial_RAM = psutil.virtual_memory().percent
                rospy.loginfo(f"Initial CPU Load: {initial_CPU:.2f}%")
                rospy.loginfo(f"Initial RAM Usage: {initial_RAM:.2f}%")
                self.start_monitoring()
                break
            rospy.sleep(0.5)  # Sleep to avoid busy-waiting

    def start_monitoring(self):
        self.start_time = time.time()  # Use time.time() for real-world start time
        rospy.Subscriber("/clock", Clock, self.clock_callback)
        self.monitoring = True
        rospy.Timer(rospy.Duration(1.0), self.update_metrics, oneshot=False)  # Call update_metrics every 1 second

    def update_metrics(self, event):
        if self.monitoring:
            self.calculate_performance_metrics()

    def calculate_performance_metrics(self):
        global initial_CPU, initial_RAM

        # Start timing the computation
        comp_start_time = time.time()

        # Simulate or calculate the actual computation task
        result = self.computation_task()  # Now using the result

        # End timing the computation
        comp_end_time = time.time()

        # Compute the time taken for the task (in milliseconds)
        comp_time = (comp_end_time - comp_start_time) * 1000.0  # Convert to milliseconds
        self.comp_times.append(comp_time)

        # RAM Usage
        current_mem_info = psutil.virtual_memory()
        mem_usage = current_mem_info.percent
        self.ram_usages.append(mem_usage)

        # Update and calculate CPU percentage
        cpu_percent = psutil.cpu_percent(interval=None)
        self.cpu_percents.append(cpu_percent)

        # Publish metrics
        self.cpu_publish.publish(Float32(cpu_percent))
        self.mem_publish.publish(Float32(mem_usage))
        self.comp_time_publish.publish(Float32(comp_time))

        # Log the metrics along with the computation result
        rospy.loginfo(f"Computation Time: {comp_time:.2f} ms")
        rospy.loginfo(f"CPU Load: {cpu_percent - initial_CPU:.2f}%")
        rospy.loginfo(f"RAM Usage: {mem_usage - initial_RAM:.2f}%")

    def computation_task(self):
        # Example computation task: Sum of the first 10 million numbers
        total_sum = 0
        for i in range(10000000):
            total_sum += i
        return total_sum  # Returning the result of the computation

    def clock_callback(self, msg):
        pass

    def stop_monitoring(self):
        """Stop monitoring and shutdown node."""
        self.monitoring = False
        rospy.signal_shutdown("Clock topic deactivated.")

    def on_shutdown(self):
        if self.start_time:
            end_time = time.time()
            elapsed_time = end_time - self.start_time

            # Calculate average computation time, CPU load, and RAM usage
            avg_comp_time = sum(self.comp_times) / len(self.comp_times) if self.comp_times else 0
            avg_cpu_percent = sum(self.cpu_percents) / len(self.cpu_percents) if self.cpu_percents else 0
            avg_ram_usage = sum(self.ram_usages) / len(self.ram_usages) if self.ram_usages else 0

            rospy.loginfo(f"Processing completed in {elapsed_time:.2f} seconds")
            rospy.loginfo(f"Average Computation Time: {avg_comp_time:.2f} ms")
            rospy.loginfo(f"Average CPU Load: {avg_cpu_percent - initial_CPU:.2f}%")
            rospy.loginfo(f"Average RAM Usage: {avg_ram_usage - initial_RAM:.2f}%")
        else:
            rospy.logwarn("No monitoring was performed.")

if __name__ == "__main__":
    rospy.loginfo(f"Initial CPU: {initial_CPU:.2f}%")
    rospy.loginfo(f"Initial RAM: {initial_RAM:.2f}%")
    try:
        monitor = PerformanceMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

