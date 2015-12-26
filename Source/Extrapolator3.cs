//  Entity Position Interpolation Code library.
//  Copyright 2006 Jon Watte, All Rights Reserved.
//  You may use this code, free of charge, for your own 
//  projects, under certain conditions ("MIT License")
//  explained in the file LICENSE included with this 
//  source code. If you do not have this file, see the 
//  web site http://www.mindcontrol.org/~hplus/epic/
//  for license conditions.

using UnityEngine;
using System.Collections;

public class Extrapolator3 {
	//! Extrapolator maintains state about updates for remote entities, and 
	//! will generate smooth guesses about where those entities will be 
	//! based on previously received data.
	//! 
	//! You create one Extrapolator per quantity you want to interpolate. 
	//! Arguments include the type to interpolate (works best with float 
	//! or double), and how many values/axes (typically 3 for positions).
	//!
	//! You then feed it updates about where your entity was ("positions") 
	//! at some point in time. Optionally, you can also give the Extrapolator
	//! information about the velocity at that point in time.
	//! After that, you can ask the Extrapolator for position values for 
	//! some given real time. Optionally, you can also ask for the velocity 
	//! at the same time.
	//!
	//! The Extrapolator will assume that an entity stops in place if it 
	//! hasn't received updates for some time about the entity. It will also 
	//! keep a running estimate of the latency and frequency of updates.
	//!
	//! Extrapolator requires a globally synchronized clock. It does not do 
	//! any management to deal with clock skew (that is the subject of a 
	//! separate class).

	//! Create the Extrapolator, initializing all fields and guesses to 0.
	public Extrapolator3(){
		Reset (0, 0, Vector3.zero);
	}

	//! When you receive data about a remote entity, call AddSample() to 
	//! tell the Extrapolator about it. The Extrapolator will throw away a 
	//! sample with a time that's before any other time it's already gotten, 
	//! and return false; else it will use the sample to improve its 
	//! interpolation guess, and return true.
	//! \param packetTime The globally synchronized time at which the 
	//! packet was sent (and thus the data valid).
	//! \param curTime The globally synchronized time at which you put 
	//! the data into the Extrapolator (i e, "now").
	//! \param pos The position sample valid for packetTime.
	//! \return true if packetTime is strictly greater than the previous 
	//! packetTime, else false.
	public bool AddSample(double packetTime, double curTime, Vector3 pos)
	{
		//  The best guess I can make for velocity is the difference between 
		//  this sample and the last registered sample.
		Vector3 vel;
		float diff = (float)(packetTime - lastPacketTime_);
		if (Mathf.Abs(diff) > 1e-4) {
			float dt = (float)(1.0 / (packetTime - lastPacketTime_));

			vel = (pos - lastPacketPos_) * dt;
		}
		else {
			vel = Vector3.zero;
		}
		return AddSample(packetTime, curTime, pos, vel);		
	}

	//! When you receive data about a remote entity, call AddSample() to 
	//! tell the Extrapolator about it. The Extrapolator will throw away a 
	//! sample with a time that's before any other time it's already gotten, 
	//! and return false; else it will use the sample to improve its 
	//! interpolation guess, and return true.
	//!
	//! If you get velocity information with your position updates, you can 
	//! make the guess that Extrapolator makes better, by passing that 
	//! information along with your position sample.
	//! \param packetTime The globally synchronized time at which the 
	//! packet was sent (and thus the data valid).
	//! \param curTime The globally synchronized time at which you put 
	//! the data into the Extrapolator (i e, "now").
	//! \param pos The position sample valid for packetTime.
	//! \return true if packetTime is strictly greater than the previous 
	//! packetTime, else false.
	//! \param vel The velocity of the entity at the time of packetTime.
	//! Used to improve the guess about entity position.
	public bool AddSample(double packetTime, double curTime, Vector3 pos, Vector3 vel)
	{
		if (!Estimates(packetTime, curTime)) {
			return false;
		}

		lastPacketPos_ = pos;
		lastPacketTime_ = packetTime;
		ReadPosition(curTime, out snapPos_);
		aimTime_ = curTime + updateTime_;
		float dt = (float)(aimTime_ - packetTime);
		snapTime_ = curTime;

		aimPos_ = pos + vel * dt;

		//  I now have two positions and two times:
		//  aimPos_ / aimTime_
		//  snapPos_ / snapTime_
		//  I must generate the interpolation velocity based on these two samples.
		//  However, if aimTime_ is the same as snapTime_, I'm in trouble. In that 
		//  case, use the supplied velocity.
		float dif = (float) (aimTime_ - snapTime_);
		if (Mathf.Abs(dif) < 1e-4) {
			snapVel_ = vel;
		}
		else {
			dt =  (float)(1.0 / (aimTime_ - snapTime_));
			snapVel_ = (aimPos_ - snapPos_) * dt;
		}
		return true;		
	}

	//! Re-set the Extrapolator's idea of time, velocity and position.
	//! \param packetTime The packet time to reset to.
	//! \param curTime The current time to reset to.
	//! \param pos The position to reset to.
	//! \note The velocity will be re-set to 0.
	public void Reset(double packetTime, double curTime, Vector3 pos)
	{
		Reset (packetTime, curTime, pos, Vector3.zero);
	}

	//! Re-set the Extrapolator's idea of time, velocity and position.
	//! \param packetTime The packet time to reset to.
	//! \param curTime The current time to reset to.
	//! \param pos The position to reset to.
	//! \param vel The velocity to reset to.
	public void Reset(double packetTime, double curTime, Vector3 pos, Vector3 vel)
	{
//		assert(packetTime <= curTime);
		lastPacketTime_ = packetTime;
		lastPacketPos_ = pos;
		snapTime_ = curTime;
		snapPos_ = pos;
		updateTime_ = curTime - packetTime;
		latency_ = updateTime_;
		aimTime_ = curTime + updateTime_;
		snapVel_= vel;
		aimPos_ = (snapPos_ + snapVel_ * (float)updateTime_);
	}

	//! Return an estimate of the interpolated position at a given global 
	//! time (which typically will be greater than the curTime passed into 
	//! AddSample()).
	//! \param forTime The time at which to interpolate the entity. It should 
	//! be greater than the last packetTime, and less than the last curTime 
	//! plus some allowable slop (determined by EstimateFreqency()).
	//! \param oPos The interpolated position for the given time.
	//! \return false if forTime is out of range (at which point the oPos 
	//! will still make sense, but movement will be clamped); true when forTime
	//! is within range.
	public bool ReadPosition(double forTime, out Vector3 oPos) 
	{
		Vector3 vel;
		return ReadPosition (forTime, out oPos, out vel);
	}

	//! Return an estimate of the interpolated position at a given global 
	//! time (which typically will be greater than the curTime passed into 
	//! AddSample()).
	//! \param forTime The time at which to interpolate the entity. It should 
	//! be greater than the last packetTime, and less than the last curTime 
	//! plus some allowable slop (determined by EstimateFreqency()).
	//! \param oPos The interpolated position for the given time.
	//! \param oVel The interpolated velocity for the given time.
	//! \return false if forTime is out of range (at which point the oPos 
	//! will still make sense, but velocity will be zero); true when forTime
	//! is within range.
	public bool ReadPosition(double forTime, out Vector3 oPos, out Vector3 oVel) {
		bool ok = true;

		//  asking for something before the allowable time?
		if (forTime < snapTime_) {
			forTime = snapTime_;
			ok = false;
		}

		//  asking for something very far in the future?
		double maxRange = aimTime_ + updateTime_;
		if (forTime > maxRange) {
			forTime = maxRange;
			ok = false;
		}

		//  calculate the interpolated position
		oVel = snapVel_;
		oPos = snapPos_ + oVel * (float)(forTime - snapTime_);
		if (!ok) {
			oVel = Vector3.zero;
		}

		return ok;		
	}

	//! \return the current estimation of latency between the sender and this
	//! interpolator. This is updated after each AddSample(), and re-set 
	//! on Reset().
	public double EstimateLatency(){
		return latency_;
	}

	//! \return the current estimation of frequency of updates that the sender 
	//! will send. This is updated after each AddSample(), and re-set on Reset().
	public double EstimateUpdateTime(){
		return updateTime_;		
	}

	private Vector3 snapPos_;
	private Vector3 snapVel_;
	private Vector3 aimPos_;
	private Vector3 lastPacketPos_;     //  only used when re-constituting velocity
	private double snapTime_;               //  related to snapPos_
	private double aimTime_;                //  related to aimPos_
	private double lastPacketTime_;         //  related to lastPacketPos_
	private double latency_;
	private double updateTime_;

	private bool Estimates(double packet, double cur)
	{
		if (packet <= lastPacketTime_) {
			return false;
		}

		//  The theory is that, if latency increases, quickly 
		//  compensate for it, but if latency decreases, be a 
		//  little more resilient; this is intended to compensate 
		//  for jittery delivery.
		double lat = cur - packet;
		if (lat < 0) lat = 0;
		if (lat > latency_) {
			latency_ = (latency_ + lat) * 0.5;
		}
		else {
			latency_ = (latency_ * 7 + lat) * 0.125;
		}

		//  Do the same running average for update time.
		//  Again, the theory is that a lossy connection wants 
		//  an average of a higher update time.
		double tick = packet - lastPacketTime_;
		if (tick > updateTime_) {
			updateTime_ = (updateTime_ + tick) * 0.5;
		}
		else {
			updateTime_ = (updateTime_ * 7 + tick) * 0.125;
		}

		return true;
	}

}
