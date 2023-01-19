#!/usr/bin/env python
# User menu to choose a card or set follower mode
while True:
    print("press  \n \
        0: Demo Triangle card\n \
        1: Card 1\n\
        2: Card 2\n \
        3: Card 3\n \
        4: Card 4\n \
        5: Card 5\n \
        6: Card 6\n \
        7: Move to starting position to load gripper\n \
        8: Follower mode \n \
        q: Quit")

    choice = input()
    if choice == "0":  
        print("Starting the Demo Card. Confirm (c) or back (b)")
        confirm = input()
        if confirm =="c" or confirm =="": 
                demotrianglecard()
        else: 
                pass
 
    elif choice == "1": 
            print("Starting Card 1. Confirm (c or enter) or back (b)")
            confirm = input()
            if confirm =="c" or confirm =="": 
                demotrianglecard()
            else: 
                pass

            card1()
    elif choice == "2":
        print("Starting Card 2")
        confirm = input()
        if confirm =="c" or confirm =="": 
            demotrianglecard()
        else: 
            pass
