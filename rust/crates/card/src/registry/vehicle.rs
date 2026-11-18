use super::Interface;
use crate::{
    core::{ApplyInput, ApplyOutput, Error, Message, Vehicle},
    firmware_query::StartupIo,
};
use openpilot_can::Packet;
use openpilot_cereal::car_capnp::car_state;

impl Vehicle for Interface {
    fn set_speedcam_context(&mut self, engaged: bool, cruise_kph: f64) {
        if let Self::Hyundai(vehicle) = self {
            vehicle.state.speedcam_engaged = engaged;
            vehicle.state.speedcam_v_cruise = cruise_kph;
        }
    }
    fn speedcam_gas_tok_consumed(&self) -> bool {
        if let Self::Hyundai(vehicle) = self {
            vehicle.state.speedcam_gas_tok_consumed
        } else {
            false
        }
    }
    fn stock_adopt_kph(&self) -> f64 {
        if let Self::Hyundai(vehicle) = self {
            vehicle.state.stock_adopt.kph
        } else {
            0.
        }
    }
    fn take_param_writes(&mut self) -> Vec<(String, Vec<u8>)> {
        if let Self::Gm(vehicle) = self {
            vehicle.take_param_writes()
        } else {
            Vec::new()
        }
    }
    fn take_diagnostics(&mut self) -> Vec<String> {
        match self {
            Self::Hyundai(vehicle) => vehicle.take_diagnostics(),
            Self::Tesla(vehicle) => vehicle.take_diagnostics(),
            Self::Mazda(vehicle) => vehicle.take_diagnostics(),
            Self::Nissan(vehicle) => vehicle.take_diagnostics(),
            Self::Chrysler(vehicle) => vehicle.take_diagnostics(),
            Self::Rivian(vehicle) => vehicle.take_diagnostics(),
            Self::Ford(vehicle) => vehicle.take_diagnostics(),
            Self::Subaru(vehicle) => vehicle.take_diagnostics(),
            Self::Toyota(vehicle) => vehicle.take_diagnostics(),
            Self::Gm(vehicle) => vehicle.take_diagnostics(),
            Self::Honda(vehicle) => vehicle.take_diagnostics(),
            Self::Volkswagen(vehicle) => vehicle.take_diagnostics(),
            Self::Body(_) | Self::Mock(_) => Vec::new(),
        }
    }
    fn take_warnings(&mut self) -> Vec<String> {
        match self {
            Self::Hyundai(vehicle) => vehicle.take_warnings(),
            Self::Body(vehicle) => vehicle.take_warnings(),
            Self::Mock(_)
            | Self::Tesla(_)
            | Self::Mazda(_)
            | Self::Nissan(_)
            | Self::Chrysler(_)
            | Self::Rivian(_)
            | Self::Ford(_)
            | Self::Subaru(_)
            | Self::Toyota(_)
            | Self::Gm(_)
            | Self::Honda(_)
            | Self::Volkswagen(_) => Vec::new(),
        }
    }
    fn take_logs(&mut self) -> Vec<crate::core::VehicleLog> {
        match self {
            Self::Hyundai(vehicle) => vehicle.take_logs(),
            Self::Tesla(vehicle) => vehicle.take_logs(),
            Self::Mazda(vehicle) => vehicle.take_logs(),
            Self::Nissan(vehicle) => vehicle.take_logs(),
            Self::Chrysler(vehicle) => vehicle.take_logs(),
            Self::Rivian(vehicle) => vehicle.take_logs(),
            Self::Ford(vehicle) => vehicle.take_logs(),
            Self::Subaru(vehicle) => vehicle.take_logs(),
            Self::Toyota(vehicle) => vehicle.take_logs(),
            Self::Gm(vehicle) => vehicle.take_logs(),
            Self::Honda(vehicle) => vehicle.take_logs(),
            Self::Volkswagen(vehicle) => vehicle.take_logs(),
            Self::Body(_) | Self::Mock(_) => Vec::new(),
        }
    }
    fn update(&mut self, packets: &[Packet], now_ns: u64) -> Result<Message, Error> {
        match self {
            Self::Body(vehicle) => vehicle.update(packets, now_ns),
            Self::Hyundai(vehicle) => Ok(vehicle.update(packets, now_ns)?),
            Self::Mock(vehicle) => vehicle.update(packets, now_ns),
            Self::Tesla(vehicle) => Ok(vehicle.update(packets, now_ns)?),
            Self::Mazda(vehicle) => Ok(vehicle.update(packets, now_ns)?),
            Self::Nissan(vehicle) => Ok(vehicle.update(packets, now_ns)?),
            Self::Chrysler(vehicle) => Ok(vehicle.update(packets, now_ns)?),
            Self::Rivian(vehicle) => Ok(vehicle.update(packets, now_ns)?),
            Self::Ford(vehicle) => Ok(vehicle.update(packets, now_ns)?),
            Self::Subaru(vehicle) => Ok(vehicle.update(packets, now_ns)?),
            Self::Toyota(vehicle) => Ok(vehicle.update(packets, now_ns)?),
            Self::Gm(vehicle) => Ok(vehicle.update(packets, now_ns)?),
            Self::Honda(vehicle) => Ok(vehicle.update(packets, now_ns)?),
            Self::Volkswagen(vehicle) => Ok(vehicle.update(packets, now_ns)?),
        }
    }
    fn init(&mut self, io: &mut impl StartupIo) -> Result<(), Error> {
        match self {
            Self::Body(vehicle) => vehicle.init(io),
            Self::Hyundai(vehicle) => Ok(vehicle.init(io)?),
            Self::Mock(vehicle) => vehicle.init(io),
            Self::Tesla(vehicle) => Ok(vehicle.init(io)?),
            Self::Mazda(vehicle) => Ok(vehicle.init(io)?),
            Self::Nissan(vehicle) => Ok(vehicle.init(io)?),
            Self::Chrysler(vehicle) => Ok(vehicle.init(io)?),
            Self::Rivian(vehicle) => Ok(vehicle.init(io)?),
            Self::Ford(vehicle) => Ok(vehicle.init(io)?),
            Self::Subaru(vehicle) => Ok(vehicle.init(io)?),
            Self::Toyota(vehicle) => Ok(vehicle.init(io)?),
            Self::Gm(vehicle) => Ok(vehicle.init(io)?),
            Self::Honda(vehicle) => Ok(vehicle.init(io)?),
            Self::Volkswagen(vehicle) => Ok(vehicle.init(io)?),
        }
    }
    fn apply(&mut self, input: ApplyInput<'_>) -> Result<ApplyOutput, Error> {
        match self {
            Self::Body(vehicle) => vehicle.apply(input),
            Self::Hyundai(vehicle) => Ok(vehicle.apply(input)?),
            Self::Mock(vehicle) => vehicle.apply(input),
            Self::Tesla(vehicle) => Ok(vehicle.apply(input)?),
            Self::Mazda(vehicle) => Ok(vehicle.apply(input)?),
            Self::Nissan(vehicle) => Ok(vehicle.apply(input)?),
            Self::Chrysler(vehicle) => Ok(vehicle.apply(input)?),
            Self::Rivian(vehicle) => Ok(vehicle.apply(input)?),
            Self::Ford(vehicle) => Ok(vehicle.apply(input)?),
            Self::Subaru(vehicle) => Ok(vehicle.apply(input)?),
            Self::Toyota(vehicle) => Ok(vehicle.apply(input)?),
            Self::Gm(vehicle) => Ok(vehicle.apply(input)?),
            Self::Honda(vehicle) => Ok(vehicle.apply(input)?),
            Self::Volkswagen(vehicle) => Ok(vehicle.apply(input)?),
        }
    }
    fn commit_state(&mut self, state: car_state::Reader<'_>) -> Result<(), Error> {
        match self {
            Self::Body(vehicle) => vehicle.commit_state(state),
            Self::Hyundai(vehicle) => Ok(vehicle.commit_state(state)?),
            Self::Mock(vehicle) => vehicle.commit_state(state),
            Self::Tesla(vehicle) => Ok(vehicle.commit_state(state)?),
            Self::Mazda(vehicle) => Ok(vehicle.commit_state(state)?),
            Self::Nissan(vehicle) => Ok(vehicle.commit_state(state)?),
            Self::Chrysler(vehicle) => Ok(vehicle.commit_state(state)?),
            Self::Rivian(vehicle) => Ok(vehicle.commit_state(state)?),
            Self::Ford(vehicle) => Ok(vehicle.commit_state(state)?),
            Self::Subaru(vehicle) => Ok(vehicle.commit_state(state)?),
            Self::Toyota(vehicle) => Ok(vehicle.commit_state(state)?),
            Self::Gm(vehicle) => Ok(vehicle.commit_state(state)?),
            Self::Honda(vehicle) => Ok(vehicle.commit_state(state)?),
            Self::Volkswagen(vehicle) => Ok(vehicle.commit_state(state)?),
        }
    }
    fn set_soft_hold(&mut self, active: i16) {
        match self {
            Self::Body(vehicle) => vehicle.set_soft_hold(active),
            Self::Hyundai(vehicle) => vehicle.set_soft_hold(active),
            Self::Mock(vehicle) => vehicle.set_soft_hold(active),
            Self::Tesla(vehicle) => vehicle.set_soft_hold(active),
            Self::Mazda(vehicle) => vehicle.set_soft_hold(active),
            Self::Nissan(vehicle) => vehicle.set_soft_hold(active),
            Self::Chrysler(vehicle) => vehicle.set_soft_hold(active),
            Self::Rivian(vehicle) => vehicle.set_soft_hold(active),
            Self::Ford(vehicle) => vehicle.set_soft_hold(active),
            Self::Subaru(vehicle) => vehicle.set_soft_hold(active),
            Self::Toyota(vehicle) => vehicle.set_soft_hold(active),
            Self::Gm(vehicle) => vehicle.set_soft_hold(active),
            Self::Honda(vehicle) => vehicle.set_soft_hold(active),
            Self::Volkswagen(vehicle) => vehicle.set_soft_hold(active),
        }
    }
}
