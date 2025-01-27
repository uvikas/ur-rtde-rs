pub const UR_SCRIPT: &str = r###"
# HEADER_BEGIN
    global reg_offset_float = # float register offset
0    global reg_offset_int = # int register offset
0    global force_mode_type = 2
    global selection_vector = [0, 0, 0, 0, 0, 0]
    global task_frame = p[0, 0, 0, 0, 0, 0]
    global wrench = [0, 0, 0, 0, 0, 0]
    global limits = [0, 0, 0, 0, 0, 0]
    global is_servoing = 0
    global is_speeding = 0
    global is_in_forcemode = 0

    global servo_target = [0, 0, 0, 0, 0, 0]
    global servo_time = 0.002
    global servo_lookahead_time = 0.1
    global servo_gain = 300

    global servoc_target = p[0, 0, 0, 0, 0, 0]
    global servoc_acceleration = 1.2
    global servoc_velocity = 0.25
    global servoc_blend = 0

    global speed_type = 0
    global speed_target = [0, 0, 0, 0, 0, 0]
    global speed_acceleration = 0.5
    global speed_time = 0.5

    global move_thrd = 0
    global move_type = 0
    global move_p = p[0, 0, 0, 0, 0, 0]
    global move_q = [0, 0, 0, 0, 0, 0]
    global move_vel = 1.2
    global move_acc = 0.25

    global stop_thrd = 0
    global stop_type = 0 # 0 = stopj, 1 = stopl
    global stop_dec = 0

    global servo_thrd = 0
    global servoc_thrd = 0
    global speed_thrd = 0
    global force_thrd = 0

    global jog_feature = 0 # 0 - base, 1 - tool, 2 - custom
    global jog_speed_pose = p[0, 0, 0, 0, 0, 0]
    global jog_custom_feature = p[0, 0, 0, 0, 0, 0]
    global jog_thrd = 0
    global jog_acc = 0.5

    global contact_direction = p[0, 0, 0, 0, 0, 0]
    global contact_step_back = 0
    global contact_thrd = 0
    global internal_cmd = 0
    
    def read_input_float_reg(register_index):
        return read_input_float_register(register_index + reg_offset_float)
    end

    def read_input_integer_reg(register_index):
        return read_input_integer_register(register_index + reg_offset_int)
    end

    def write_output_float_reg(register_index, value):
        write_output_float_register(register_index + reg_offset_float, value)
    end

    def write_output_integer_reg(register_index, value):
        write_output_integer_register(register_index + reg_offset_int, value)
    end

    def exec_move_path():
        textmsg("exec_move_path")
        # inject move path
    end


    global async_wr_count = 0 # is incremented each time the register is written
    global async_op_id = 0 # is incremented each time a new async operation is started

    # Write async progress register
    # This function provides extended progress information in an async progress 
    # status register. The following bits are used
    #
    # | Bit 31 | Bit 30 - 24 | Bit 23 | Bit 9 - 22     | Bit 15  | Bit 14 - 0 |
    # +--------+-------------+--------+----------------+---------+------------+
    # | rsv    | async_op_id | rsv    | async_wr_count | running | progress   | 
    #
    # running: 1 - async op. running, 0 - async op. finished
    # progress: progress of running async. operation
    #
    def write_async_progress_register(value):
        # we use 7 bits for the change count - that means 127 is our maximum
        async_wr_count = async_wr_count + 1
        if async_wr_count > 127:
            async_wr_count = 0
        end

        # 0 means new async operation started - we increment the opration id
        # we use 7 bits for the operation id - that means 127 is our maximum
        if value == 0:
            async_op_id = async_op_id + 1
            if async_op_id > 127:
                async_op_id = 0
            end
        end

        reg_value = async_wr_count * 65536 # shift change counter to bits 16 - 23
        reg_value = reg_value + (async_op_id * 16777216) # shift opration id to bits 24 - 30

        # if an async operation is active, we set the bit 15 (value 32768) and
        # store the progress value into the lower 15 bit
        if value >= 0:
            # ensure that the progress value only uses 15 bit bacause bit 15
            # is used for indication of runnin state
            if value > 32767:
                value = 32767
            end
            reg_value = reg_value + value + 32768 # store the progress in the lower 16 bits            
        end
        textmsg("async_wr_count: ", async_wr_count)
        textmsg("async_op_id: ", async_op_id)
        write_output_integer_reg(2, reg_value)
      end

      def signal_async_progress(value):
        write_async_progress_register(value)
      end
  
      def signal_async_operation_started():
        write_async_progress_register(0) # 0 indicates operation started
      end
  
      def signal_async_operation_finished():
        write_async_progress_register(-1) # negative values indicate operation finished
      end
  
      def reset_async_progress():
        signal_async_operation_finished()
      end

    # asynchronous stop thread
    # stopping a move with a low deceleration may take quite some time, so
    # async execution may prevent blocking the caller
    thread stop_thread():
      textmsg("stop_thread started")
      signal_async_operation_started()
      while (True):
          if stop_type == 0:
              stopj(stop_dec)
          elif stop_type == 1:
              stopl(stop_dec)
          end
          enter_critical
          stop_thrd = 0
          textmsg("stop_thread finished")
          exit_critical
          break
      end
      signal_async_operation_finished()
    end

    thread move_thread():
        textmsg("move_thread started")
        signal_async_operation_started()
        while (True):
            if move_type == 0 or move_type == 1:
                movej(move_q, a=move_acc, v=move_vel)
            elif move_type == 2:
                movel(move_p, a=move_acc, v=move_vel)
            elif move_type == 3:
                movel(move_q, a=move_acc, v=move_vel)
            elif move_type == 4:
                exec_move_path()
            end
            enter_critical
            move_thrd = 0
            textmsg("move_thread finished")
            exit_critical
            break
        end
        signal_async_operation_finished()
    end

    def stop_async_move():
        enter_critical
        if move_thrd != 0:
            textmsg("stopping async movement - killing move_thrd")
            kill move_thrd
            move_thrd = 0
            # If the move thread is killed, it cannot trigger the 
            # signal_async_operation_finished() signal - so we do it here
            signal_async_operation_finished()
        end
        exit_critical
    end

    # Execute an sync or async stopl or stopj command.
    def exec_stopl_stopj(cmd, message):
      deceleration_rate = read_input_float_reg(0)
      async = read_input_integer_reg(1)
      textmsg(message, async)
      # if a stop thread is already running, we kill it so that we can restart it
      # with the new parameters in the code below
      stop_move = True
      enter_critical
      if stop_thrd != 0:
          stop_move = False # if a stop thread is running then someone already called async stop before
          textmsg("killing stop_thrd")
          kill stop_thrd
          stop_thrd = 0
          signal_async_operation_finished()
      end
      exit_critical

      # We only need to stop robot movement, if it has not been stopped by a
      # previous running async stop command
      if stop_move == True:
        stop_async_move()
      end

      # If this is an async stop, then start the stop thread now. If not, then
      # we stop synchronously
      if async == 1:
          enter_critical
          stop_type = 1
          stop_dec = deceleration_rate
          exit_critical
          stop_thrd = run stop_thread()
      else:
          if stop_type == 0:
            stopj(deceleration_rate)
          elif stop_type == 1:
            stopl(deceleration_rate)
          end
      end
    end

    thread force_thread():
        while (True):
            force_mode(task_frame, selection_vector, wrench, force_mode_type, limits)
            sync()
        end
    end

    thread speed_thread():
        while (True):
            enter_critical
            type = speed_type
            target = speed_target
            acceleration = speed_acceleration
            time = speed_time
            exit_critical
            if type == 0:
                if time > 0:
                    speedl(target, a=acceleration, t=time)
                else:
                    speedl(target, a=acceleration)
                end
            else:
                if time > 0:
                    speedj(target, a=acceleration, t=time)
                else:
                    speedj(target, a=acceleration)
                end
            end
        end
    end

    thread servo_thread():
        while (True):
            enter_critical
            q = servo_target
            dt = servo_time
            lh_time = servo_lookahead_time
            g = servo_gain
            exit_critical
            servoj(q, t=dt, lookahead_time=lh_time, gain=g)
        end
    end

    thread servoc_thread():
        while (True):
            servoc(servoc_target, a=servoc_acceleration, v=servoc_velocity, r=servoc_blend)
        end
    end

    thread contact_thread():
      textmsg("contact_thread started")
      enter_critical
      contact_step_back = 0
      dir = contact_direction
      exit_critical

      textmsg("direction:", dir)
      use_speed_vector = pose_is_null(dir)
      if use_speed_vector:
          textmsg("Using speed vector")
      end

      step_back = 1
      # First we wait until there is not contact
      while step_back > 0:
          if use_speed_vector:
              step_back = tool_contact(direction=get_target_tcp_speed())
          else:
              step_back = tool_contact(direction=dir)
          end
          sync()
      end

      # Now we can wait for a contact
      while step_back == 0:
          if use_speed_vector:
              step_back = tool_contact(direction=get_target_tcp_speed())
          else:
              step_back = tool_contact(direction=dir)
          end
          sync()
      end
      textmsg("step_back:", step_back)
      textmsg("target_tcp_speed: ", get_target_tcp_speed())
      enter_critical
      contact_thrd = 0
      contact_step_back = step_back
      internal_cmd = 254 # delegate processing of contact detection to main thread
      exit_critical
      textmsg("contact_thread finished")
    end

    def q_from_input_float_registers(register_index):
      q = [0, 0, 0, 0, 0, 0]
      q[0] = read_input_float_reg(register_index + 0)
      q[1] = read_input_float_reg(register_index + 1)
      q[2] = read_input_float_reg(register_index + 2)
      q[3] = read_input_float_reg(register_index + 3)
      q[4] = read_input_float_reg(register_index + 4)
      q[5] = read_input_float_reg(register_index + 5)
      return q
    end

    def pose_from_input_float_registers(register_index):
      pose = p[0, 0, 0, 0, 0, 0]
      pose[0] = read_input_float_reg(register_index + 0)
      pose[1] = read_input_float_reg(register_index + 1)
      pose[2] = read_input_float_reg(register_index + 2)
      pose[3] = read_input_float_reg(register_index + 3)
      pose[4] = read_input_float_reg(register_index + 4)
      pose[5] = read_input_float_reg(register_index + 5)
      return pose
    end

    def pose_to_output_float_registers(register_index, pose):
      write_output_float_reg(register_index + 0, pose[0])
      write_output_float_reg(register_index + 1, pose[1])
      write_output_float_reg(register_index + 2, pose[2])
      write_output_float_reg(register_index + 3, pose[3])
      write_output_float_reg(register_index + 4, pose[4])
      write_output_float_reg(register_index + 5, pose[5])
    end

    def q_to_output_float_registers(register_index, q):
      write_output_float_reg(register_index + 0, q[0])
      write_output_float_reg(register_index + 1, q[1])
      write_output_float_reg(register_index + 2, q[2])
      write_output_float_reg(register_index + 3, q[3])
      write_output_float_reg(register_index + 4, q[4])
      write_output_float_reg(register_index + 5, q[5])
    end

    # Returs a pose that contains only the translation part of a given pose
    def get_pose_translation(pose):
      return p[pose[0], pose[1], pose[2], 0, 0, 0]
    end
  
    # Returns a pose that contains only the rotation part of a given pose
    def get_pose_rotation(pose):
      return p[0, 0, 0, pose[3], pose[4], pose[5]]
    end

    # scales the given pose by the given factor
    def scale_pose(p, factor):
      p[0] = p[0] * factor
      p[1] = p[1] * factor
      p[2] = p[2] * factor
      p[3] = p[3] * factor
      p[4] = p[4] * factor
      p[5] = p[5] * factor
      return p
    end

    def pose_is_null(p):
      sum = p[0] + p[1] + p[2] + p[3] + p[4] + p[5]
      return (sum < 0.0000000001)
    end

    # convert tool_speed expressed in the frame_wrt_base in the robot base frame
    # examples :
    # speed_wrt_base([0.020, 0.020,0,0,0,0], plane_1)
    # speedl(get_speed_wrt_base([0,0,0.02,0,0,1.57], get_actual_tcp_pose()),a=1,t=0.5,aRot=4))
  
    #### input arguments
    # tool_speed : [Tx, Ty, Tz, Rx, Ry, Rz] spatial vector list - Txyz [m/s]), Rxyz [rad/s]
    # frame_wrt_base : pose
  
    #### output arguments
    # speed_wrt_base : [Tx, Ty, Tz, Rx, Ry, Rz] spatial vector list - Txyz [m/s]), Rxyz [rad/s]
    def get_speed_wrt_base(tool_speed, frame_wrt_base):    
      # Translationnal speed vector calculus
      T_wrt_frame=p[tool_speed[0],tool_speed[1],tool_speed[2],0,0,0]
      T_wrt_base_raw= pose_trans (frame_wrt_base, T_wrt_frame)
      T_wrt_base=pose_sub(T_wrt_base_raw,frame_wrt_base)
    
      # Rotationnal speed vector calculus
      R_wrt_frame=p[tool_speed[3],tool_speed[4],tool_speed[5],0,0,0]
      R_wrt_frame_abs=norm([R_wrt_frame[0],R_wrt_frame[1],R_wrt_frame[2]])
      if R_wrt_frame_abs != 0:  # test if zero vector
        R_wrt_base_raw=pose_trans(frame_wrt_base, R_wrt_frame)
        R_wrt_base=pose_sub(R_wrt_base_raw, frame_wrt_base)
      else:
        R_wrt_base=p[0,0,0,0,0,0]
      end
    
      # Concatenate T and R
      speed_wrt_base_list=[T_wrt_base[0],T_wrt_base[1],T_wrt_base[2],R_wrt_base[0],R_wrt_base[1],R_wrt_base[2]]
      return speed_wrt_base_list   
    end

    
    # The jog thread provides jogging functionality similar to the one you
    # find in the teach pendant.
    thread jog_speed_thread():
      textmsg("jog_speed_thread started")
      enter_critical
      feature = jog_feature
      exit_critical

      # Initialize target_pose depending on feature
      if feature == 0: # base feature
        frame_wrt_base = p[0, 0, 0, 0, 0, 0]
      elif feature == 1: # tool feature
        frame_wrt_base = get_actual_tcp_pose()
      elif feature == 2: # custom feature
        enter_critical
        frame_wrt_base = jog_custom_feature
        exit_critical
      end

      while (True):
        enter_critical
        speed_pose = jog_speed_pose
        exit_critical
        speed_wrt_base = get_speed_wrt_base(speed_pose, frame_wrt_base)
        speedl(speed_wrt_base, jog_acc, t=0.01)
      end
    end

    def signal_ready():
        write_output_integer_reg(0, 1)
    end

    def signal_done_with_cmd():
        write_output_integer_reg(0, 2)
    end

    def rtde_cmd():
      cmd = read_input_integer_reg(0)
      # only if command is 0 - that means we are idle, we process internal
      # commands
      if cmd == 0:
          enter_critical
          cmd = internal_cmd
          internal_cmd = 0
          exit_critical
      end
      return cmd
    end

    def process_cmd(cmd):
        if cmd == 1:
            textmsg("movej")
            q = q_from_input_float_registers(0)
            velocity = read_input_float_reg(6)
            acceleration = read_input_float_reg(7)
            async = read_input_integer_reg(1)
            textmsg("Target q:")
            textmsg(q)
            stop_async_move()
            if async == 1:
                enter_critical
                move_type = 0
                move_q = q
                move_acc = acceleration
                move_vel = velocity
                exit_critical
                move_thrd = run move_thread()
            else:
                movej(q, a=acceleration, v=velocity)
                textmsg("movej done")
            end
        elif cmd == 2:
            textmsg("movej_ik")
            pose = pose_from_input_float_registers(0)
            velocity = read_input_float_reg(6)
            acceleration = read_input_float_reg(7)
            async = read_input_integer_reg(1)
            textmsg("Target pose:")
            textmsg(pose)
            q = get_inverse_kin(pose)
            textmsg("Target q:")
            textmsg(q)
            stop_async_move()
            if async == 1:
                enter_critical
                move_type = 1
                move_q = q
                move_acc = acceleration
                move_vel = velocity
                exit_critical
                move_thrd = run move_thread()
            else:
                movej(q, a=acceleration, v=velocity)
                textmsg("movej_ik done")
            end
        elif cmd == 3:
            textmsg("movel")
            pose = pose_from_input_float_registers(0)
            velocity = read_input_float_reg(6)
            acceleration = read_input_float_reg(7)
            async = read_input_integer_reg(1)
            textmsg("Target pose:")
            textmsg(pose)
            stop_async_move()
            if async == 1:
                enter_critical
                move_type = 2
                move_p = pose
                move_acc = acceleration
                move_vel = velocity
                exit_critical
                move_thrd = run move_thread()
            else:
                movel(pose, a=acceleration, v=velocity)
                textmsg("movel done")
            end
        elif cmd == 4:
            textmsg("movel_fk")
            q = q_from_input_float_registers(0)
            velocity = read_input_float_reg(6)
            acceleration = read_input_float_reg(7)
            async = read_input_integer_reg(1)
            textmsg("Target q:")
            textmsg(q)
            stop_async_move()
            if async == 1:
                enter_critical
                move_type = 3
                move_q = q
                move_acc = acceleration
                move_vel = velocity
                exit_critical
                move_thrd = run move_thread()
            else:
                movel(q, a=acceleration, v=velocity)
                textmsg("movel_fk done")
            end
        elif cmd == 6:
            # force_mode
            enter_critical
            force_mode_type = read_input_integer_reg(1)
            selection_vector[0] = read_input_integer_reg(2)
            selection_vector[1] = read_input_integer_reg(3)
            selection_vector[2] = read_input_integer_reg(4)
            selection_vector[3] = read_input_integer_reg(5)
            selection_vector[4] = read_input_integer_reg(6)
            selection_vector[5] = read_input_integer_reg(7)

            task_frame = pose_from_input_float_registers(0)
            wrench = q_from_input_float_registers(6)
            limits = q_from_input_float_registers(12)
            exit_critical

            if is_in_forcemode == 0:
                is_in_forcemode = 1
                if force_thrd == 0:
                    global force_thrd = run force_thread()
                end
            end
        elif cmd == 7:
            textmsg("force_mode_stop")
            enter_critical
            is_in_forcemode = 0
            kill force_thrd
            force_thrd = 0
            end_force_mode()
            stopl(10)
            exit_critical
            textmsg("force_mode stopped")
        elif cmd == 8:
            textmsg("zero_ftsensor")
            zero_ftsensor()
            textmsg("ftsensor zeroed")
        elif cmd == 9:
            # speedJ
            qd = q_from_input_float_registers(0)

            enter_critical
            speed_type = 1
            speed_acceleration = read_input_float_reg(6)
            speed_time = read_input_float_reg(7)
            speed_target = qd
            exit_critical

            if is_speeding == 0:
                enter_critical
                is_speeding = 1
                exit_critical
                if speed_thrd == 0:
                    global speed_thrd = run speed_thread()
                end
            end
        elif cmd == 10:
            # speedL
            xd = q_from_input_float_registers(0)
            enter_critical
            speed_type = 0
            speed_acceleration = read_input_float_reg(6)
            speed_time = read_input_float_reg(7)
            speed_target = xd
            exit_critical

            if is_speeding == 0:
                is_speeding = 1
                if speed_thrd == 0:
                    global speed_thrd = run speed_thread()
                end
            end
        elif cmd == 11:
            # servoJ
            q = q_from_input_float_registers(0)
            velocity = read_input_float_reg(6)
            acceleration = read_input_float_reg(7)

            enter_critical
            servo_target = q
            servo_time = read_input_float_reg(8)
            servo_lookahead_time = read_input_float_reg(9)
            servo_gain = read_input_float_reg(10)
            exit_critical

            if is_servoing == 0:
                is_servoing = 1
                if servo_thrd == 0:
                    global servo_thrd = run servo_thread()
                end
            end

        elif cmd == 12:
            # servoC
            pose = pose_from_input_float_registers(0)
            velocity = read_input_float_reg(6)
            acceleration = read_input_float_reg(7)
            blend = read_input_float_reg(8)

            enter_critical
            servoc_target = pose
            servoc_acceleration = acceleration
            servoc_velocity = velocity
            servoc_blend = blend
            exit_critical

            if is_servoing == 0:
                is_servoing = 1
                if servoc_thrd == 0:
                    global servoc_thrd = run servoc_thread()
                end
            end

        elif cmd == 15:
            textmsg("speed_stop")
            deceleration_rate = read_input_float_reg(0)
            enter_critical
            is_speeding = 0
            kill speed_thrd
            speed_thrd = 0
            if speed_type == 0:
                stopl(deceleration_rate)
            else:
                stopj(deceleration_rate)
            end
            exit_critical
            textmsg("speed_stop done")
        elif cmd == 16:
            textmsg("servo_stop")
            deceleration_rate = read_input_float_reg(0)
            enter_critical
            is_servoing = 0
            kill servo_thrd
            kill servoc_thrd
            servo_thrd = 0
            servoc_thrd = 0
            exit_critical
            stopl(deceleration_rate)
            textmsg("servo_stop done")
        elif cmd == 17:
            textmsg("set_payload")
            mass = read_input_float_reg(0)
            cog_x = read_input_float_reg(1)
            cog_y = read_input_float_reg(2)
            cog_z = read_input_float_reg(3)
            cog = [cog_x, cog_y, cog_z]
            if cog_x == 0 and cog_y == 0 and cog_z == 0:
                set_payload(mass, get_target_payload_cog())
            else:
                set_payload(mass, cog)
            end
            textmsg("active payload:")
            textmsg(get_target_payload())
            textmsg("set_payload done")
        elif cmd == 18:
            textmsg("teach_mode")
            teach_mode()
            textmsg("teach_mode done")
        elif cmd == 19:
            textmsg("end_teach_mode")
            end_teach_mode()
            textmsg("end_teach_mode done")
        elif cmd == 20:
            textmsg("force_mode_set_damping")
            damping = read_input_float_reg(0)
            force_mode_set_damping(damping)
            textmsg("force_mode_set_damping done")
        elif cmd == 21:
            textmsg("force_mode_set_gain_scaling")
            scaling = read_input_float_reg(0)
            force_mode_set_gain_scaling(scaling)
            textmsg("force_mode_set_gain_scaling done")
        elif cmd == 24:
            pose = pose_from_input_float_registers(0)
            velocity = read_input_float_reg(6)
            acceleration = read_input_float_reg(7)
            q = get_inverse_kin(pose)

            enter_critical
            servo_target = q
            servo_time = read_input_float_reg(8)
            servo_lookahead_time = read_input_float_reg(9)
            servo_gain = read_input_float_reg(10)
            exit_critical

            if is_servoing == 0:
                is_servoing = 1
                if servo_thrd == 0:
                    global servo_thrd = run servo_thread()
                end
            end
        elif cmd == 25:
            # tool_contact
            direction = pose_from_input_float_registers(0)
            time_steps = tool_contact(direction)
            write_output_integer_reg(1, time_steps)
        elif cmd == 26:
            # get_steptime
            step_time = get_steptime()
            write_output_float_reg(0, step_time)
        elif cmd == 27:
            # get_actual_joint_positions_history
            steps = read_input_integer_reg(1)
            joint_positions_history = get_actual_joint_positions_history(steps)
            q_to_output_float_registers(0, joint_positions_history)
        elif cmd == 28:
            textmsg("get_target_waypoint")
            target_waypoint = get_target_waypoint()
            pose_to_output_float_registers(0, target_waypoint)
            textmsg("get_target_waypoint done")
        elif cmd == 29:
            textmsg("set_tcp")
            pose = pose_from_input_float_registers(0)
            set_tcp(pose)
            textmsg("set_tcp done")
        elif cmd == 30:
            textmsg("get_inverse_kin_args")
            x = pose_from_input_float_registers(0)
            qnear = q_from_input_float_registers(6)
            maxPositionError = read_input_float_reg(12)
            maxOrientationError = read_input_float_reg(13)

            q = get_inverse_kin(x, qnear, maxPositionError, maxOrientationError)
            q_to_output_float_registers(0, q)
            textmsg("get_inverse_kin done")
        elif cmd == 31:
            textmsg("protective_stop")
            protective_stop()
            textmsg("protective_stop done")
        elif cmd == 33:
            exec_stopl_stopj(cmd, "stopl async=")
            textmsg("stopl done")
        elif cmd == 34:
            exec_stopl_stopj(cmd, "stopj async=")
            textmsg("stopj done")
        elif cmd == 35:
            textmsg("set_watchdog")
            # Setup watchdog for the RTDE communication
            watchdog_min_frequency = read_input_float_reg(0)
            if reg_offset_int == 0:
                rtde_set_watchdog("input_int_register_0", watchdog_min_frequency, "stop")
            elif reg_offset_int == 24:
                rtde_set_watchdog("input_int_register_24", watchdog_min_frequency, "stop")
            else:
                rtde_set_watchdog("input_int_register_0", watchdog_min_frequency, "stop")
            end
            textmsg("set_watchdog done")
        elif cmd == 36:
            textmsg("is_pose_within_safety_limits")
            pose = pose_from_input_float_registers(0)
            safe_pose = is_within_safety_limits(pose)
            if safe_pose == True:
               write_output_integer_reg(1, 1)
            else:
               write_output_integer_reg(1, 0)
            end
            textmsg("is_pose_within_safety_limits done")
        elif cmd == 37:
            textmsg("is_joints_within_safety_limits")
            q = q_from_input_float_registers(0)
            safe_q = is_within_safety_limits(q)
            if safe_q == True:
               write_output_integer_reg(1, 1)
            else:
               write_output_integer_reg(1, 0)
            end
            textmsg("is_joints_within_safety_limits done")
        elif cmd == 38:
            # get_joint_torques
            torques = get_joint_torques()
            q_to_output_float_registers(0, torques)
        elif cmd == 39:
            textmsg("pose_trans")
            p_from = pose_from_input_float_registers(0)
            p_from_to = pose_from_input_float_registers(6)
            p = pose_trans(p_from, p_from_to)
            pose_to_output_float_registers(0, p)
            textmsg("pose_trans done")
        elif cmd == 40:
            textmsg("get_tcp_offset")
            tcp_offset = get_tcp_offset()
            textmsg(tcp_offset)
            pose_to_output_float_registers(0, tcp_offset)
            textmsg("get_tcp_offset done")
        elif cmd == 41:
              textmsg("start_jog")
              enter_critical
              jog_speed_pose = pose_from_input_float_registers(0)
              jog_feature = read_input_float_reg(6)
              jog_acc = read_input_float_reg(7)
              jog_custom_feature = pose_from_input_float_registers(8)
              if jog_thrd == 0:
                jog_thrd = run jog_speed_thread()
              end
              exit_critical
              textmsg("jog_feature: ", jog_feature)
              textmsg("jog_custom_feature: ", jog_custom_feature)
              textmsg("jog_acc: ", jog_acc)
              textmsg("start_jog done")
        elif cmd == 42:
              textmsg("stop_jog")
              enter_critical
              if jog_thrd != 0:
                textmsg("stopping jogging - killing jog_thrd")
                kill jog_thrd
                jog_thrd = 0
              end
              exit_critical
              stopl(1.2)
              textmsg("stop_jog done")
        elif cmd == 43:
              textmsg("get_forward_kinematics_default")
              forward_kin = get_forward_kin()
              textmsg(forward_kin)
              pose_to_output_float_registers(0, forward_kin)
              textmsg("get_forward_kinematics_default done")
        elif cmd == 44:
              textmsg("get_forward_kinematics_args")
              q = q_from_input_float_registers(0)
              tcp_offset = pose_from_input_float_registers(6)
              forward_kin = get_forward_kin(q, tcp_offset)
              textmsg(forward_kin)
              pose_to_output_float_registers(0, forward_kin)
              textmsg("get_forward_kinematics_args done")
        elif cmd == 45:
              textmsg("move_path")
              async = read_input_integer_reg(1)
              textmsg("async: ", async)
              stop_async_move()
              if async == 1:
                  enter_critical
                  move_type = 4 # move_path
                  exit_critical
                  move_thrd = run move_thread()
              else:
                  exec_move_path()
                  textmsg("move_path done")
              end
        elif cmd == 46:
              textmsg("get_inverse_kin_default")
              x = pose_from_input_float_registers(0)
              q = get_inverse_kin(x)
              q_to_output_float_registers(0, q)
              textmsg("get_inverse_kin_default done")
        elif cmd == 47:
              textmsg("is_steady")
              robot_is_steady = is_steady()
              if robot_is_steady == True:
                 write_output_integer_reg(1, 1)
              else:
                 write_output_integer_reg(1, 0)
              end
              textmsg("is_steady done")
        elif cmd == 51:
              textmsg("move_until_contact")
              xd = q_from_input_float_registers(0)
              contact_dir = pose_from_input_float_registers(6)
              acc = read_input_float_reg(12)
              while True:
                 step_back = tool_contact(direction=contact_dir)
                 if step_back <= 0:
                    # Continue moving with specified speed vector
                    speedl(xd, acc, t=get_steptime())
                 else:
                    # Contact detected!
                    # Get q for when the contact was first seen
                    q = get_actual_joint_positions_history(step_back)
                    # Stop the movement
                    stopl(3)
                    # Move to the initial contact point
                    movel(q)
                    break
                 end
              end
              textmsg("move_until_contact done")
        elif cmd == 52:
              textmsg("freedrive_mode")
              free_axes = [1, 1, 1, 1, 1, 1]
              free_axes[0] = read_input_integer_reg(1)
              free_axes[1] = read_input_integer_reg(2)
              free_axes[2] = read_input_integer_reg(3)
              free_axes[3] = read_input_integer_reg(4)
              free_axes[4] = read_input_integer_reg(5)
              free_axes[5] = read_input_integer_reg(6)

              freedrive_feature = pose_from_input_float_registers(0)
              freedrive_mode(freeAxes=free_axes, feature=freedrive_feature)
              textmsg("freedrive_mode done")
        elif cmd == 53:
              textmsg("end_freedrive_mode")
              end_freedrive_mode()
              textmsg("end_freedrive_mode done")
        elif cmd == 54:
              textmsg("get_freedrive_status")
              freedrive_status = get_freedrive_status()
              write_output_integer_reg(1, freedrive_status)
              textmsg("get_freedrive_status done")
        elif cmd == 56:
              textmsg("ft_rtde_input_enable")
              enable = False
              enable_val = read_input_integer_reg(1)
              if enable_val == 1:
                  enable = True
              elif enable_val == 0:
                  enable = False
              end
              sensor_mass = read_input_float_reg(0)
              sensor_measuring_offset = [0, 0, 0]
              sensor_measuring_offset[0] = read_input_float_reg(1)
              sensor_measuring_offset[1] = read_input_float_reg(2)
              sensor_measuring_offset[2] = read_input_float_reg(3)
              sensor_cog = [0, 0, 0]
              sensor_cog[0] = read_input_float_reg(4)
              sensor_cog[1] = read_input_float_reg(5)
              sensor_cog[2] = read_input_float_reg(6)
              ft_rtde_input_enable(enable, sensor_mass, sensor_measuring_offset, sensor_cog)
              textmsg("ft_rtde_input_enable done")
        elif cmd == 57:
              textmsg("enable_external_ft_sensor")
              enable = False
              enable_val = read_input_integer_reg(1)
              if enable_val == 1:
                  enable = True
              elif enable_val == 0:
                  enable = False
              end
              sensor_mass = read_input_float_reg(0)
              sensor_measuring_offset = [0, 0, 0]
              sensor_measuring_offset[0] = read_input_float_reg(1)
              sensor_measuring_offset[1] = read_input_float_reg(2)
              sensor_measuring_offset[2] = read_input_float_reg(3)
              sensor_cog = [0, 0, 0]
              sensor_cog[0] = read_input_float_reg(4)
              sensor_cog[1] = read_input_float_reg(5)
              sensor_cog[2] = read_input_float_reg(6)
              enable_external_ft_sensor(enable, sensor_mass, sensor_measuring_offset, sensor_cog)
              textmsg("enable_external_ft_sensor done")
        elif cmd == 58:
              textmsg("get_actual_tool_flange_pose")
              flange_pose = get_actual_tool_flange_pose()
              q_to_output_float_registers(0, flange_pose)
              textmsg("get_actual_tool_flange_pose done")
        elif cmd == 59:
              textmsg("set_gravity")
              gravity_direction = [0, 0, 0]
              gravity_direction[0] = read_input_float_reg(0)
              gravity_direction[1] = read_input_float_reg(1)
              gravity_direction[2] = read_input_float_reg(2)
              set_gravity(gravity_direction)
              textmsg("set_gravity done")
        elif cmd == 60:
              textmsg("get_inverse_kin_has_solution_default")
              x = pose_from_input_float_registers(0)
              has_solution = get_inverse_kin_has_solution(x)
              if has_solution == True:
                 write_output_integer_reg(1, 1)
              else:
                 write_output_integer_reg(1, 0)
              end
              textmsg("get_inverse_kin_has_solution_default done")
        elif cmd == 61:
              textmsg("get_inverse_kin_has_solution_args")
              x = pose_from_input_float_registers(0)
              qnear = q_from_input_float_registers(6)
              maxPositionError = read_input_float_reg(12)
              maxOrientationError = read_input_float_reg(13)

              has_solution = get_inverse_kin_has_solution(x, qnear, maxPositionError, maxOrientationError)
              if has_solution == True:
                  write_output_integer_reg(1, 1)
              else:
                  write_output_integer_reg(1, 0)
              end
              textmsg("get_inverse_kin_has_solution_args done")
            elif cmd == 62:
              textmsg("start_contact_detection")
              enter_critical
              if contact_thrd != 0:
                  textmsg("killing contact_thrd")
                  kill contact_thrd
                  contact_thrd = 0
              end
              exit_critical
              contact_direction = pose_from_input_float_registers(0)
              contact_thrd = run contact_thread()
              textmsg("start_contact_detection done")
          elif cmd == 63:
              textmsg("stop_contact_detection")
              enter_critical
              if contact_thrd != 0:
                  textmsg("killing contact_thrd")
                  kill contact_thrd
                  contact_thrd = 0
              end
              exit_critical
              write_output_integer_reg(1, contact_step_back)
              textmsg("stop_contact_detection done")
          elif cmd == 64:
              textmsg("read_contact_detection")
              enter_critical
              step_back = contact_step_back
              exit_critical
              write_output_integer_reg(1, step_back)
              textmsg("read_contact_detection done")
          elif cmd == 65:
              textmsg("set_target_payload")
              mass = read_input_float_reg(0)
              cog_x = read_input_float_reg(1)
              cog_y = read_input_float_reg(2)
              cog_z = read_input_float_reg(3)
              cog = [cog_x, cog_y, cog_z]
              inertia = [0, 0, 0, 0, 0, 0]
              inertia[0] = read_input_float_reg(4)
              inertia[1] = read_input_float_reg(5)
              inertia[2] = read_input_float_reg(6)
              inertia[3] = read_input_float_reg(7)
              inertia[4] = read_input_float_reg(8)
              inertia[5] = read_input_float_reg(9)
              if cog_x == 0 and cog_y == 0 and cog_z == 0:
                 set_target_payload(mass, get_target_payload_cog(), inertia)
              else:
                 set_target_payload(mass, cog, inertia)
              end
              textmsg("active payload:")
              textmsg(get_target_payload())
              textmsg("active payload inertia matrix")
              textmsg(get_target_payload_inertia())
              textmsg("set_target_payload done")
          elif cmd == 254: # internal command
              textmsg("cmd == 254 - contact detected") 
              stop_async_move()
              # Get q for when the contact was first seen
              q = get_actual_joint_positions_history(contact_step_back)
              # Stop the movement
              stopl(3)
              # Move to the initial contact point
              movel(q)
              textmsg("cmd == 254 - contact detected done")
        elif cmd == 255:
              textmsg("Received stop script!")
        end

        if cmd != 255:
              signal_done_with_cmd()
        end

        return cmd != 255
    end

# HEADER_END

# NODE_CONTROL_LOOP_BEGINS

        ###################################
        # RTDE Control script - Main loop #
        ###################################
        textmsg("RTDE Control Script Loaded (14.02.2024-12:23)")

        # Initialize gain and damping for force mode to a more stable default
        force_mode_set_gain_scaling(0.5)
        force_mode_set_damping(0.025)

        keep_running = True
        executing_cmd = False
        reset_async_progress()
        signal_ready()

        while keep_running:
            cmd = rtde_cmd()
            if cmd == 24 or cmd == 11 or cmd == 9 or cmd == 10 or cmd == 6 or cmd == 25 or cmd == 26 or cmd == 27 or cmd == 38 or cmd == 58:
                # for realtime commands simply process and signal ready.
                keep_running = process_cmd(cmd)
                signal_ready()
            else:
                # regular mode
                if cmd == 0:
                    executing_cmd = False
                    signal_ready()
                else:
                    if not executing_cmd:
                        keep_running = process_cmd(cmd)
                    end
                    executing_cmd = True
                end
            end

            sync()
        end
        textmsg("RTDE Control Script Terminated")
# NODE_CONTROL_LOOP_ENDS
"###;
