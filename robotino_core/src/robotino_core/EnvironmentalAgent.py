import matplotlib.pyplot as plt


class EnvironmentalAgent:

    def __init__(self, segment_id):
        self.id = segment_id
        self.blocked = False
        self.rho = 0.5  # Percent evaporation of pheromone (0..1, default=0.8)
        self.t0 = 0.1  # Minimal pheromone
        self.reservations = {}

    def __iter__(self):
        if self.reservations:
            return iter([reservation for reservation in self.reservations.values()])
        else:
            return iter([])

    def check_slot(self, slot, agv_id):

        # Check all available slots of other robots
        available_slots = self.get_available_slots(slot, agv_id)

        # Take first available slot and adapt the end time to the required end time
        first_available_slot = available_slots[0]
        first_available_slot = (first_available_slot[0], slot[1])

        # Compute delay
        delay = first_available_slot[0] - slot[0]
        return first_available_slot, delay

    def reserve_slot(self, slot, agv_id):

        # Get slot
        reservation_time = slot[0]
        duration = slot[1]

        # Check overlap with reservations of other robots
        end_time = reservation_time + duration
        valid = True
        made_reservations = [reservation for reservation in self.reservations.values() if
                             not reservation.agv_id == agv_id]
        for reservation in made_reservations:
            if reservation_time <= reservation.start < end_time or reservation_time < reservation.start \
                    + reservation.duration <= end_time:
                valid = False
                break
        if valid:
            self.reservations[reservation_time, duration] = Reservation(agv_id, reservation_time, duration)
            # print("Slot (" + str(reservation_time) + ', ' + str(duration) + ") reserved for agv " + str(agv_id))
        else:
            print("Could not add slot (" + str(reservation_time) + ', ' + str(duration) + ") of agv " + str(agv_id))

    def remove_reservation(self, reservation_time, duration):
        del self.reservations[reservation_time, duration]

    def remove_reservations(self, agv_id):
        reservations = [reservation for reservation in self.reservations.values() if reservation.agv_id == agv_id]
        for reservation in reservations:
            del self.reservations[reservation.start, reservation.duration]

    def remove_schedule(self):
        self.reservations = {}

    def get_available_slots(self, slot, agv_id):

        # Get slot information
        reservation_time = slot[0]
        duration = slot[1]

        # Get all reservations from the requested reservation_time for all other robots
        slots = [(value.start, value.start + value.duration) for key, value in self.reservations.items() if
                 value.start + value.duration > reservation_time and not value.agv_id == agv_id]

        # Get free slots
        free_slots = []
        if slots:
            # Free slot from requested reservation time till start time of eariest reservation
            if slots[0][0] - reservation_time >= duration:
                free_slots.append((reservation_time, slots[0][0]))
            # Free intermediate slots
            for start, end in ((slots[i][1], slots[i + 1][0]) for i in range(len(slots) - 1)):
                if end - start >= duration:
                    free_slots.append((start, end - start))
            # Free slot from last reservation time till infinity
            free_slots.append((slots[-1][1], float('inf')))
        else:
            free_slots.append((reservation_time, float('inf')))
        return free_slots

    def evaporate(self):
        for reservation in self.reservations.values():
            reservation.pheromone = max(self.t0, reservation.pheromone * (1 - self.rho))
            if reservation.pheromone == self.t0:
                self.remove_reservation(reservation.start, reservation.duration)

    def plot(self, ax=None):
        if ax is None:
            fig, ax = plt.subplots(1, 1)
        for reservation in self.reservations.values():
            ax.plot(reservation.start, reservation.agv_id, 'k.')
            ax.plot(reservation.start + reservation.duration / 2, reservation.agv_id, 'r.')
            ax.plot(reservation.start + reservation.duration, reservation.agv_id, 'k.')
            ax.plot([reservation.start, reservation.start + reservation.duration],
                    [reservation.agv_id, reservation.agv_id], 'k-')
        ax.set_title("Segment " + self.id)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("agv id's")


class Reservation:

    def __init__(self, agv_id, reservation_time, duration, pheromone=1.0):
        self.agv_id = agv_id
        self.start = reservation_time
        self.duration = duration
        self.pheromone = pheromone
