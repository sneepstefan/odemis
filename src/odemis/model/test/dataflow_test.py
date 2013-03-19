#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Created on 17 Jul 2012

@author: Éric Piel

Copyright © 2012-2013 Éric Piel, Delmic

This file is part of Odemis.

Odemis is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 2 of the License, or (at your option) any later version.

Odemis is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with Odemis. If not, see http://www.gnu.org/licenses/.
'''
from Pyro4.core import oneway
from odemis import model
import Queue
import logging
import pickle
import threading
import time
import unittest

# unused, alternative implementation
class EventQueue(object):
    """
    Event implementation using queues. Avoids blocking but latency ~1ms.
    """
    def __init__(self):
        self._queues = {} # listener -> Queue (of boolean)
    
    def subscribe(self, listener, callback=None):
        # TODO: handle when changing callback or queue
        if (listener not in self._queues or 
            (callback and self._queues[listener] != callback) or
            (callback is None and not isinstance(self._queues[listener], Queue.Queue))):
            if callback:
                self._queues[listener] = callback
            else:
                self._queues[listener] = Queue.Queue(10) # bounded to detect over-run
    
    def unsubscribe(self, listener):
        try:
            if isinstance(self._queues[listener], Queue.Queue):
                self._queues[listener].put(False) # event to say it's being unsubscribed
            else:
                self._queues[listener](False)
            del self._queues[listener]
        except KeyError:
            # listener not in the queue -> it's fine
            pass

    def trigger(self):
        for l, q in self._queues.items():
            if isinstance(q, Queue.Queue):
                if q.full():
                    logging.warning("Over-run for listener %r of event %r", l, q)
                    # TODO: skip, or ensure it still gets it eventually?
                    continue # don't delay the rest of the listeners
    #            q.put(True)
                q.put(time.time())
                time.sleep(0) # yield, so that the other threads get it faster
            else:
                q(time.time())
    
    def wait(self, listener, timeout=None):
        """
        Wait for the event to trigger
        returns (boolean): True if the event has happened, False if timeout or 
         unsubscribed (unsynchronized).
        Note: works only if the subscription was without callback
        """
        try:
            return self._queues[listener].get(timeout=timeout)
        except (Queue.Empty, KeyError):
            return False
    
    def clear(self, listener):
        # TODO: if listener is None, clear all queues?
        if isinstance(self._queues[listener], Queue.Queue):
            self._queues[listener].clear()
            
class Event(object):
    """
    Simple implementation of simplistic event interface. Callback directly each
    subscriber. Low latency, but blocking in each subscriber.
    Pretty similar to a VigilantAttribute, but:
     * doesn't contain value (so no unit, range either)
     * every notify matters, so none should be discarded ever.
    """ 
    def __init__(self):
        self._listeners = set() # callback (None -> None)
    
    def subscribe(self, listener):
        """
        Register a callback function to be called when the Event is changed
        listener (function): callback function which takes no argument and return nothing
        """
        assert callable(listener)
        self._listeners.add(model.WeakMethod(listener))
        
    def unsubscribe(self, listener):
        self._listeners.discard(model.WeakMethod(listener))

    def notify(self):
        for l in self._listeners.copy():
            try:
                l(time.time()) # DEBUG only => no parameter normally
            except model.WeakRefLostError:
                self.unsubscribe(l)
    
class SimpleDataFlow(model.DataFlow):
    # very basic dataflow
    def __init__(self, *args, **kwargs):
        model.DataFlow.__init__(self, *args, **kwargs)
        self._thread_must_stop = threading.Event()
        self._thread = None
        self.startAcquire = Event() # triggers when the acquisition starts
    
    def _thread_main(self):
        i = 0
        # generate a stupid array every 0.1s
        while not self._thread_must_stop.wait(0.1):
            self.startAcquire.notify()
            data = model.DataArray([[i, 0],[0, 0]], metadata={"a": 1, "num": i})
            i += 1
            self.notify(data)
        self._thread_must_stop.clear()
    
    def start_generate(self):
        # if there is already a thread, wait for it to finish before starting a new one
        if self._thread:
            self._thread.join()
            assert not self._thread_must_stop.is_set()
            self._thread = None
        
        # create a thread
        self._thread = threading.Thread(target=self._thread_main, name="flow thread")
        self._thread.start()

    def stop_generate(self):
        assert self._thread
        assert not self._thread_must_stop.is_set()
        # we don't wait for the thread to stop fully
        self._thread_must_stop.set()
    
class SynchronizableDataFlow(model.DataFlow):
    # very basic dataflow
    def __init__(self, *args, **kwargs):
        model.DataFlow.__init__(self, *args, **kwargs)
        self._thread_must_stop = threading.Event()
        self._thread = None
        self._sync_event = None
        self.max_lat = []
        self._got_event = threading.Event()
    
    @oneway
    def _on_trigger(self, triggert):
        latency = time.time() - triggert
        self.max_lat.append(latency)
        self._got_event.set()
    
    def _wait_event_or_stop_cb(self):
        """
        return True if must stop, False otherwise
        """
        while not self._thread_must_stop.is_set():
            event = self._sync_event 
            if event is None:
                return False
            # In practice, this would be a hardware signal, not an threading.event!
            if self._got_event.wait(timeout=0.1):
                self._got_event.clear()
                return False
        return True
        
    def _wait_event_or_stop(self):
        """
        return True if must stop, False otherwise
        """
        while not self._thread_must_stop.is_set():
            event = self._sync_event 
            if event is None:
                return False
            # timeout doesn't need to be very small, just often enough to check
            # for must_stop. However, it seems a big timeout will lead to bigger
            # latency (seems to do sleeps of 10% of the timeout)
            # => at best: ~1ms latency (= scheduling latency?)
            # with callback we get ~50us
            triggert = event.wait(self, timeout=0.0001)
            if triggert:
                latency = time.time() - triggert
                self.max_lat.append(latency)
                return False
        return True
        
    def _thread_main(self):
        i = 0
        # generate a stupid array every time we receive an event
        while not self._thread_must_stop.is_set():
            time.sleep(0.01) # bit of "initialisation" time
            
            must_stop = self._wait_event_or_stop_cb()
            if must_stop:
                break
#            if self._sync_event:
#                self._sync_event.wait(self)
            
#            time.sleep(1) #DEBUG: for test over-run
            data = model.DataArray([[i, 0],[0, 0]], metadata={"a": 2, "num": i})
            i += 1
            self.notify(data)
        self._thread_must_stop.clear()
    
    def start_generate(self):
        # if there is already a thread, wait for it to finish before starting a new one
        if self._thread:
            self._thread.join()
            assert not self._thread_must_stop.is_set()
            self._thread = None
        
        # create a thread
        self._thread = threading.Thread(target=self._thread_main, name="flow thread")
        self._thread.start()

    def stop_generate(self):
        assert self._thread
        assert not self._thread_must_stop.is_set()
        # we don't wait for the thread to stop fully
        self._thread_must_stop.set()
    
    def synchronizedOn(self, event):
        if self._sync_event == event:
            return
        
        if self._sync_event:
            self._sync_event.unsubscribe(self._on_trigger)
        
        self._sync_event = event
        if self._sync_event:
            self._sync_event.subscribe(self._on_trigger)

        
class TestDataFlow(unittest.TestCase):
    
#    @unittest.skip("simple")
    def test_dataarray_pickle(self):
        darray = model.DataArray([[1, 2],[3, 4]], metadata={"a": 1})
        jar = pickle.dumps(darray)
        up_darray = pickle.loads(jar)
        self.assertEqual(darray.data, up_darray.data, "data is different after pickling")
        self.assertEqual(darray.metadata, up_darray.metadata, "metadata is different after pickling")
        self.assertEqual(up_darray.metadata["a"], 1)

#    @unittest.skip("simple")
    def test_df_subscribe_get(self):
        self.df = SimpleDataFlow()
        self.size = (2,2)
        
        number = 5
        self.left = number
        self.df.subscribe(self.receive_data)
        
        time.sleep(0.2)
        
        # get one image: should be shared with the subscribe
        im = self.df.get()
        
        # get a second image: also shared
        im = self.df.get()

        self.assertEqual(im.shape, self.size)
        self.assertIn("a", im.metadata)
        
        for i in range(number):
            # end early if it's already finished
            if self.left == 0:
                break
            time.sleep(0.2) # 0.2s per image should be more than enough in any case
        
        self.assertEqual(self.left, 0)
    
#    @unittest.skip("simple")
    def test_df_double_subscribe(self):
        self.df = SimpleDataFlow()
        self.size = (2,2)
        number, number2 = 8, 3
        self.left = number
        self.df.subscribe(self.receive_data)
        
        time.sleep(0.2) # long enough to be after the first data
        self.left2 = number2
        self.df.subscribe(self.receive_data2)
        
        for i in range(number):
            # end early if it's already finished
            if self.left == 0:
                break
            time.sleep(0.2) # 0.2s should be more than enough in any case
        
        self.assertEqual(self.left2, 0) # it should be done before left
        self.assertEqual(self.left, 0)

    def receive_data(self, dataflow, data):
        """
        callback for df
        """
        self.assertEqual(data.shape, self.size)
        self.assertIn("a", data.metadata)
#        print "Received an image"
        self.left -= 1
        if self.left <= 0:
            dataflow.unsubscribe(self.receive_data)


    def receive_data2(self, dataflow, data):
        """
        callback for df 
        """
        self.assertEqual(data.shape, self.size)
        self.assertIn("a", data.metadata)
        self.left2 -= 1
        if self.left2 <= 0:
            dataflow.unsubscribe(self.receive_data2)


    def test_synchronized_df(self):
        self.dfe = SimpleDataFlow()
        self.dfs = SynchronizableDataFlow()
        self.dfs.synchronizedOn(self.dfe.startAcquire)
        
        self.size = (2,2)
        number = 30
        self.left = number + 10
        self.dfs.subscribe(self.receive_data)
        
        time.sleep(0.2) # long enough to be after the first data
        # ensure the dfs is hasn't generated anything yet
        self.assertEqual(self.left, number + 10)
        
        # start the eventfull df
        self.left2 = number
        self.dfe.subscribe(self.receive_data2)
        
        for i in range(number):
            # end early if it's already finished
            if self.left2 == 0:
                break
            time.sleep(0.2) # 0.2s should be more than enough in any case
        
        self.assertEqual(self.left2, 0) # it should be done before left
        self.assertEqual(self.left, 10)
        
        # make sure we can unsubscribe even if synchronized and waiting on event
        self.dfs.unsubscribe(self.receive_data) 
        self.dfs.synchronizedOn(None)
        print self.dfs.max_lat, max(self.dfs.max_lat)
        time.sleep(0.1)
        self.assertEqual(self.left, 10)

    def test_non_synchronized_df(self):
        self.df = SynchronizableDataFlow()
        self.df.synchronizedOn(None)
        
        self.size = (2,2)
        number = 3
        self.left = number
        self.df.subscribe(self.receive_data)
        
        for i in range(number):
            # end early if it's already finished
            if self.left == 0:
                break
            time.sleep(0.2) # 0.2s should be more than enough in any case
        
        self.assertEqual(self.left, 0)
        
if __name__ == "__main__":
    unittest.main()
